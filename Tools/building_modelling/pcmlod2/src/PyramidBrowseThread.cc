
/*
                  Copyright 2010 University of Twente
 
       This file is part of the Mapping libraries and tools, developed
  for research, education and projects in photogrammetry and laser scanning.

  The Mapping libraries and tools are free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License,
                   or (at your option) any later version.

 The Mapping libraries and tools are distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
          along with the Mapping libraries and tools.  If not, see
                      <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------*/


#include <math.h>
#include "PyramidBrowseThread.h"

PyramidBrowseThread::PyramidBrowseThread(QObject *parent) : QThread(parent)
{
  restart = false;
  abort   = false;
  new_points = new LaserPoints();
  transfer_points = new LaserPoints();
}

PyramidBrowseThread::~PyramidBrowseThread()
{
  mutex.lock();
  abort = true;
  condition.wakeOne();
  mutex.unlock();
  wait();
}

void PyramidBrowseThread::UpdateRequest(const DataBounds2D &bounds,
                                        double max_point_spacing,
                                        LaserPoints *current_points,
                                        LaserPyramid *pyramid,
                                        int maximum_number_of_points)
{
  QMutexLocker locker(&mutex);

  // Copy data to class variables
  this->bounds            = bounds;
  this->max_point_spacing = max_point_spacing;
  this->current_points    = current_points;
  this->current_tiles     = pyramid->UsedTiles();
  this->pyramid           = pyramid;
  this->max_num_pts       = maximum_number_of_points;

  // (Re-)start process
  if (!isRunning()) {
    if (!new_points->empty()) new_points->ErasePoints();
    if (!transfer_points->empty()) transfer_points->ErasePoints();
    start(LowPriority);
  }
  else {
    restart = true;
    condition.wakeOne();
  }
}

void PyramidBrowseThread::TransferNewSelection(LaserPoints *current_points,
                                               LaserPyramid *pyramid)
{
  bool         verbose = false;
  QMutexLocker locker(&mutex);
  
  if (verbose) printf("Transfering data to main thread\n");
  current_points->swap(*transfer_points);
  current_end_of_tile.swap(transfer_end_of_tile);
  pyramid->UsedTiles()->swap(transfer_tiles);
}

void PyramidBrowseThread::run()
{
  LaserPyramid                 *pyramid;
  double                       max_point_spacing;
  LaserPoints                  *current_points;
  DataBounds2D                 bounds;
  LaserTilePtrVector           selected_tiles, new_tiles;
  LaserTilePtrVector::iterator tile;
  int                          new_tile_index, current_tile_index, tile_size,
                               max_num_pts;
  LaserPoints::iterator        new_point_index, current_point_index;
  bool                         done, changed, verbose=false, save_new_data;
  int                          tile_sum;
  std::vector <int>            new_end_of_tile;

  forever {
    // Copy class variables to local variables
    mutex.lock();
    bounds            = this->bounds;
    max_point_spacing = this->max_point_spacing;
    current_points    = this->current_points;
    pyramid           = this->pyramid;
    max_num_pts       = this->max_num_pts;
    restart           = false;
    mutex.unlock();
    
// Adapted code from function LaserPyramid::UpdateSelection(...)
    
    // Select the tiles within the bounds
    selected_tiles = pyramid->SelectTiles(bounds, max_point_spacing);
    if (abort) return;
  
    // Check if there are any different tiles
    changed = false;
    for (tile=current_tiles->begin(); tile!=current_tiles->end() && !changed; tile++)
      if (!selected_tiles.Contains(*tile)) changed = true;
    for (tile=selected_tiles.begin(); tile!=selected_tiles.end() && !changed; tile++)
      if (!current_tiles->Contains(*tile)) changed = true;
      
    // Put thread to sleep again if there is no change
    if (!changed) {
      mutex.lock();
      condition.wait(&mutex);
      restart = true; // Don't continue with the code below
      mutex.unlock();
    }

    // Copy points from the transfer list that can be re-used
    save_new_data = false;
    if (!restart) {
      if (verbose) printf("Checking transfer points to be re-used\n");
      tile_sum = 0;
      new_points->ErasePoints();
      new_end_of_tile.erase(new_end_of_tile.begin(), new_end_of_tile.end());
      new_tiles.Erase();
      new_tile_index  = -1;
      // If all selected tiles are on the transfer list swap with the new list
      changed = false;
      for (tile=selected_tiles.begin();
           tile!=selected_tiles.end() && !changed; tile++) {
        if (!transfer_tiles.Contains(*tile)) changed = true;
      }
      if (!changed && !restart && !abort && !transfer_tiles.empty()) {
        if (verbose) printf("Re-using %d tiles\n", (int) transfer_tiles.size());
        new_points->swap(*transfer_points);
        new_end_of_tile.swap(transfer_end_of_tile);
        new_tiles.swap(transfer_tiles);
        new_tile_index = new_tiles.size()-1;
        tile_sum = new_points->size();
        save_new_data = true;
      }
      else if (verbose) printf("No tiles re-used from transfer list\n");
    }
    
    // Copy points from the previous selection that can be re-used
    if (!restart) {
      if (verbose) printf("Checking current points to be re-used\n");
      for (tile=current_tiles->begin(), current_tile_index=0;
           tile!=current_tiles->end() && !restart && !abort;
           tile++, current_tile_index++) {
        if (selected_tiles.Contains(*tile) && !new_tiles.Contains(*tile)) {
          new_tiles.push_back(*tile);
          new_tile_index++;
          if (current_tile_index == 0) {
            current_point_index = current_points->begin();
            tile_size = current_end_of_tile[0] + 1;
          }
          else {
            current_point_index = current_points->begin() +
                              current_end_of_tile[current_tile_index-1] + 1;
            tile_size = current_end_of_tile[current_tile_index] -
                        current_end_of_tile[current_tile_index-1];
          }
          tile_sum += tile_size;
          if (verbose) printf("Copying data of current tile %s\n", (*tile)->Name());
          new_points->insert(new_points->end(), current_point_index,
                             current_point_index+tile_size-1);
          if (new_tile_index == 0) new_end_of_tile.push_back(tile_size - 1);
          else 
            new_end_of_tile.push_back(new_end_of_tile[new_tile_index-1] +
                                      tile_size);
        }
      }
    }
    
    // Allow to stop here
    if (abort) return;

    // Add points of new tiles
    if (!restart) {
      done = false;
      for (tile=selected_tiles.begin();
           tile!=selected_tiles.end() && !done && !restart && !abort;
           tile++) {
        if (!new_tiles.Contains(*tile)) {
          // Read points of new tile
          if (verbose) printf("Reading new tile %s\n", (*tile)->Name());
          if (!pyramid->ReadTile(**tile)) {
            printf("Error reading points of tile %s\n", (*tile)->Name());
            return;
          }
          // Check the selection size limit
          if ((int) (new_points->size() + (*tile)->size()) > max_num_pts) {
           (*tile)->ErasePoints();
            done = true;
           break;
          };
          // Add the points and tile information
          new_points->insert(new_points->end(),
                             (*tile)->begin(), (*tile)->end());
          new_tiles.push_back(*tile);
          new_tile_index++;
          tile_sum += (*tile)->size();
          if (new_tile_index == 0)
            new_end_of_tile.push_back((*tile)->size() - 1);
          else
            new_end_of_tile.push_back(new_end_of_tile[new_tile_index-1] + (*tile)->size());
          (*tile)->ErasePoints();
          save_new_data = true;
        }
      }
    }

    // Allow to stop here
    if (abort) return;

    if (!restart || save_new_data) {
      // Swap all new information to the transfer buffers
      transfer_tiles.swap(new_tiles);
      transfer_end_of_tile.swap(new_end_of_tile);
      transfer_points->swap(*new_points);    
    
      // Erase local data
      new_tiles.Erase();
      selected_tiles.Erase();
      new_end_of_tile.erase(new_end_of_tile.begin(), new_end_of_tile.end());
      new_points->ErasePoints();

      if (!restart) {
        if (verbose) printf("Emitting NewSelectionReady signal with %d points\n",
                            (int) transfer_points->size());
        emit NewSelectionReady(transfer_points->size());

        mutex.lock();
        if (verbose) printf("Done in run(), putting thread to sleep\n");
        condition.wait(&mutex);
        mutex.unlock();
      }
    }
  }
  printf("End of run(), should not get here! restart %d abort %d\n", restart, abort);
}

/*
--------------------------------------------------------------------------------
                Remove information on previously used tiles
--------------------------------------------------------------------------------
*/

void PyramidBrowseThread::SelectedTileRange(int &min_row, int &max_row,
                                            int &min_column, int &max_column,
                                            int &level) const
{
  LaserTilePtrVector::const_iterator tile;
  
  min_row = min_column = INT_MAX;
  max_row = max_column = INT_MIN;
  for (tile=current_tiles->begin(); tile!=current_tiles->end(); tile++) {
    if ((*tile)->TileRow() < min_row) min_row = (*tile)->TileRow();
    if ((*tile)->TileRow() > max_row) max_row = (*tile)->TileRow();
    if ((*tile)->TileColumn() < min_column) min_column = (*tile)->TileColumn();
    if ((*tile)->TileColumn() > max_column) max_column = (*tile)->TileColumn();
  }
  if (pyramid != NULL) level = pyramid->UsedLevel();
  else level = -1;
}

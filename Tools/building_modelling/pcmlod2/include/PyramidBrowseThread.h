
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


#ifndef PYRAMIDBROWSETHREAD_H
#define PYRAMIDBROWSETHREAD_H

#include <stdio.h>
#include <stdlib.h>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include "LaserPyramid.h"

class PyramidBrowseThread : public QThread 
{
  Q_OBJECT

  private:

    /// Mutex
    QMutex mutex;
  
    /// Wait condition
    QWaitCondition condition;
    
    /// Restart switch
    bool restart;
    
    /// Abort switch
    bool abort;
    
    /// New laser point selection
    LaserPoints *new_points;
    
    /// Laser points to be transferred to main thread
    LaserPoints *transfer_points;
    
    /// Data bounds for new selection
    DataBounds2D bounds;
    
    /// Pointer to pyramid
    LaserPyramid *pyramid;
    
    /// Pointer to current laser point selection
    LaserPoints *current_points;

    /// List of currently used tiles
    LaserTilePtrVector *current_tiles;
    
    /// Current pyramid level
    int current_level;
    
    /// List of transfer tiles
    LaserTilePtrVector transfer_tiles;
    
    /// Maximum point spacing
    double max_point_spacing;
    
    /// Maximum number of points
    int max_num_pts;
    
    /// End of tile indices in current point selection
    std::vector <int> current_end_of_tile;

    /// End of tile indices in transfer point selection
    std::vector <int> transfer_end_of_tile;

    
  public:

    /// Constructor
    PyramidBrowseThread(QObject *parent=NULL);

    /// Default destructor
    ~PyramidBrowseThread();

    /// Restart updating of pyramid
    void UpdateRequest(const DataBounds2D &bounds, double max_point_spacing,
                       LaserPoints *current_points,
                       LaserPyramid *pyramid,
                       int maximum_number_of_points);
                  
    void TransferNewSelection(LaserPoints *current_points,
                              LaserPyramid *pyramid);
    
    void SelectedTileRange(int &min_row, int &max_row,
                           int &min_column, int &max_column, int &level) const;

  protected:

    /// The main function to update the pyramid
    void run();
    
  signals:
    
    /// Update is ready
    void NewSelectionReady(int);
};
#endif // PYRAMIDBROWSETHREAD_H

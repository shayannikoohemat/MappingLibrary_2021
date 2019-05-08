
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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <limits.h>
#include <QMessageBox>
#include <QFileDialog>
#include <QColorDialog>
#include "PointCloudMapper.h"

void PointCloudMapper::ChangeBackGroundColour()
{
  QColor new_colour = QColorDialog::getColor(canvas->BackGroundColour(), this);
  if (new_colour.isValid()) canvas->SetBackGroundColour(new_colour);
}

bool PointCloudMapper::SetBackGroundImage(BackGroundType clicked_item,
                                          PCMWindow *window)
{
  int  item, selected_item;
  bool update_bounds = !window->Canvas()->HasData(), success;

  // Clicking an already selected data type implies unselecting
  selected_item = clicked_item;
  if (window->Type() == PCMMain &&
      show_background_actions[selected_item]->isChecked())
    selected_item = NoBackGroundImage;

  // Remove all textures
  window->Canvas()->ClearTextureData(false);

  switch (selected_item) {
    case HeightImage: // Height texture
      success = GenerateHeightTexture(window);
      if (success) 
        window->Canvas()->AddTextureData(texture_images[HeightImage],
                                         &texture_bounds[HeightImage], false);
      break;

    case ShadedHeightImage: // Shaded height texture
      success = GenerateShadedHeightTexture(window);
      if (success)
        window->Canvas()->AddTextureData(texture_images[ShadedHeightImage],
                                         &texture_bounds[ShadedHeightImage],
                                         false);
      break;

    case OrthoImage: // Ortho image
      success = (texture_images[OrthoImage] != NULL);
      if (success)
        window->Canvas()->AddTextureData(texture_images[OrthoImage],
                                         &texture_bounds[OrthoImage], false);
      else
        QMessageBox::information(window, "Error", "No ortho image available!");
      break;

    case NoBackGroundImage: // No texture
      success = true;
      break;

    default:
      success = false;
      break;
  }

  // Set the new menu and button states
  if (!success) selected_item = 0; // No background after failure
  for (item=0; item<4; item++) {
    if (window->Type() == PCMMain)
      show_background_actions[item]->setChecked(item == selected_item);
/* TODO: Check if the above line replaced the commented code
      background_menu->setItemChecked(id_background[item],
                                      item == selected_item);
    if (item > 0) window->SetShowImageButton((BackGroundType) item, 
                                             item == selected_item);
*/
  }

  if (update_bounds) Canvas()->InitialiseTransformation();
  window->Canvas()->update(); // Just refresh canvas
  return success;
}

bool PointCloudMapper::GenerateHeightTexture(PCMWindow *window)
{
  double        pixelsize, min_pixelsize;
  int           num_rows, num_columns, num_tex_rows, num_tex_columns;

  if (laser_points.size() == 0) {
    QMessageBox::information(window, "Error", "No laser data available!");
    return false;
  }

  // Determine bounds and pixel size
  laser_points.DeriveDataBounds(0);
  min_pixelsize = max(laser_points.Bounds().XRange(),
                      laser_points.Bounds().YRange()) / 2046; // 2048
  pixelsize = max(0.1, min_pixelsize);

  // Set range such that the image sizes are powers of 2
  num_rows = (int) (laser_points.Bounds().YRange() / pixelsize);
  if (num_rows < laser_points.Bounds().YRange() / pixelsize) num_rows++;
  num_tex_rows = 2;
  while (num_tex_rows < num_rows) num_tex_rows *= 2;
  laser_points.Bounds().SetMaximumY(laser_points.Bounds().Minimum().Y() +
    num_tex_rows * pixelsize - 0.1 * pixelsize);

  num_columns = (int) (laser_points.Bounds().XRange() / pixelsize);
  if (num_columns < laser_points.Bounds().XRange() / pixelsize) num_columns++;
  num_tex_columns = 2;
  while (num_tex_columns < num_columns) num_tex_columns *= 2;
  laser_points.Bounds().SetMaximumX(laser_points.Bounds().Minimum().X() +
    num_tex_columns * pixelsize - 0.1 * pixelsize);

  laser_points.Bounds().SetMinimumZ(-1.0);

  // Check if there already is a height texture image with the same bounds
  if (texture_images[HeightImage] != NULL)
    if (laser_points.Bounds().Bounds3D() == texture_bounds[HeightImage])
      return true;

  // Delete the old texture image (both in normal and OpenGL format)
  if (laser_points.ImagedHeight() != NULL)
    laser_points.ImagedHeight()->DeleteImage();
  if (texture_images[HeightImage] != NULL)
    texture_images[HeightImage]->DeleteImage();
  
  // Generate the height image
  laser_points.CreateImage(pixelsize, VFF_TYP_1_BYTE, HeightData,
                             3, max(pixelsize * 5, 4.0));

  // Convert to OpenGL format
  texture_images[HeightImage] = laser_points.ImagedHeight()->ConvertToOpenGL();

  // Copy the bounds
  texture_bounds[HeightImage] = laser_points.Bounds().Bounds3D();

  return true;
}

bool PointCloudMapper::GenerateShadedHeightTexture(PCMWindow *window)
{
  Image *shaded_image;

  // Make sure there is height texture
  if (!GenerateHeightTexture(window)) return false;

  // Check if there already is a shaded height texture with the same bounds
  if (texture_images[ShadedHeightImage] != NULL)
    if (texture_bounds[ShadedHeightImage] == texture_bounds[HeightImage])
      return true;

  // Delete old OpenGL texture
  if (texture_images[ShadedHeightImage] != NULL)
    texture_images[ShadedHeightImage]->DeleteImage();
  
  // Derive the shaded image
  shaded_image = laser_points.ImagedHeight()->ShadedImage();

  // Convert to OpenGL format
  texture_images[ShadedHeightImage] = shaded_image->ConvertToOpenGL();
  shaded_image->DeleteImage(); // No longer needed

  // Copy the bounds
  texture_bounds[ShadedHeightImage] = texture_bounds[HeightImage];
  return true;
}

void PointCloudMapper::LoadBackGroundImage()
{
  QString   filename;
  Image     *background_image;
  int       success;
  ImageGrid *grid;

  // Get the image file name
  filename = QFileDialog::getOpenFileName(this, "Select ortho image",
                                          QString(), "Images (*.xv)");
  if (filename.isEmpty()) return;

  // Read the image
  background_image = new Image(filename.toLatin1(), &success);
  if (!success || background_image == NULL) {
    QMessageBox::information(this, "Error", QString("Cannot read image file ") +
                             filename);
    return;
  }

  // Get the image grid file name
  filename = QFileDialog::getOpenFileName(this, "Select grid definition file",
                                          QString(),
                                          "Grid definition (*.grid)");
  if (filename.isEmpty()) return;
  
  // Read the image grid
  grid = new ImageGrid(filename.toLatin1(), &success);
  if (!success || grid == NULL) {
    QMessageBox::information(this, "Error",
                             QString("Cannot read image grid file ") +
                             filename);
    return;
  }

  // Delete old OpenGL texture
  if (texture_images[OrthoImage] != NULL)
    texture_images[OrthoImage]->DeleteImage();
  
  // Convert to OpenGL format
  texture_images[OrthoImage] = background_image->ConvertToOpenGL();
  
  // Set the bounds
  texture_bounds[OrthoImage].Initialise();
  texture_bounds[OrthoImage].SetMinimumX(grid->XOffset());
  texture_bounds[OrthoImage].SetMaximumY(grid->YOffset());
  texture_bounds[OrthoImage].SetMaximumX(grid->XOffset() +
                                      texture_images[OrthoImage]->NumColumns() *
                                         grid->Pixelsize());
  texture_bounds[OrthoImage].SetMinimumY(grid->YOffset() -
                                         texture_images[OrthoImage]->NumRows() *
                                         grid->Pixelsize());
  texture_bounds[OrthoImage].SetMinimumZ(-1.0);
  
  // Delete image and grid structures
  background_image->DeleteImage();
  delete background_image;
  delete grid;

  // Set this background image
  SetBackGroundImage(NoBackGroundImage, PCMWindowPtr()); // To remove old image
  if (texture_images[OrthoImage] != NULL)
    SetBackGroundImage(OrthoImage, PCMWindowPtr());
}


/*
                 Copyright 2018 University of Twente
 
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



/*
--------------------------------------------------------------------------------
 Collection of functions for class IMUReadings

 Initial creation
 Author : George Vosselman
 Date   : 09-12-2018

 Update #1
 Author : 
 Date   : 
 Changes: 

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "IMUReadings.h"
#include "LaserDataTypes.h"
#include "Database4Cpp.h"
#include "BNF_io.h"
#include "stdmath.h"

/*
--------------------------------------------------------------------------------
                        Copy constructor and assignment
--------------------------------------------------------------------------------
*/

IMUReadings::IMUReadings(const IMUReadings &r)
{
  Initialise();
  LaserMetaFileReference() = r.LaserMetaFileReference();
  SetIMUFile(r.imu_file);
  start_time = r.start_time;
  end_time   = r.end_time;
}

IMUReadings IMUReadings::operator = (const IMUReadings &r)
{
  if (this == &r) return *this;  // Check for self assignment
  if (!empty()) erase(begin(),end());
  if (!r.empty()) insert(begin(), r.begin(), r.end());
  LaserMetaFileReference() = r.LaserMetaFileReference();
  SetIMUFile(r.imu_file);
  start_time = r.start_time;
  end_time   = r.end_time;
  return(*this);
}

/*
--------------------------------------------------------------------------------
                     Initialise the IMU readings
--------------------------------------------------------------------------------
*/

void IMUReadings::Initialise()
{
  imu_file = NULL;
  start_time = end_time = -1.0;
  LaserMetaFile::Initialise();
}

void IMUReadings::ReInitialise()
{
  if (!empty()) erase(begin(), end());
  if (imu_file) {free(imu_file);  imu_file = NULL;}
  start_time = end_time = -1.0;
  LaserMetaFile::ReInitialise();
}

/*
--------------------------------------------------------------------------------
                          Read IMU readings from file
--------------------------------------------------------------------------------
*/

int IMUReadings::Read(const char *filename)
{
  if (imu_file) free(imu_file);
  imu_file = (char *) malloc(strlen(filename) + 1);
  strcpy(imu_file, filename);
  return(Read());
}

int IMUReadings::Read()
{
  FILE *fd;
  int  file_id, num_bytes, offset, num_readings, num_alloc, reading_size,
       buffer_size, num_bytes_left, index;
  char *header, *ch;
  double reading[7];
  unsigned char *buffer;
    
  // Open the file
#ifdef windows
  if ((fd = Open_Compressed_File(imu_file, "rb")) == NULL) {
#else
  if ((fd = Open_Compressed_File(imu_file, "r")) == NULL) {
#endif
    fprintf(stderr, "Could not open IMU file %s\n", imu_file);
    return(0);
  }

  // Clear the old data
  if (!empty()) erase(begin(), end());

  // Read all header bytes  
  header = (char *) malloc(1025 * sizeof(char));
  if (header == NULL) {
    printf("IMUReadings::Read : Error allocating header memory\n");
    return -1;
  }
  num_bytes = fread(header, sizeof(char), 1024, fd);
  if (num_bytes != 1024) {
    printf("Error reading header of IMU file %s\n", imu_file);
    printf("%d bytes read instead of expected 1024 bytes\n", num_bytes);    
    return -1;
  }
  
  // Check file id
  memcpy((void *) &file_id, (const void *) header, sizeof(int));
  offset = sizeof(int);
  if (file_id != LASER_IMU) {
    printf("Error in file id of IMU file %s\n", imu_file);
 	return(-1);
  }

  // Extract other header information
  memcpy((void *) &num_readings, (const void *) (header+offset), sizeof(int));
  free(header);
  printf("num_readings is %d\n", num_readings);

  // Memory allocation
  num_alloc = 500000;
  if (num_readings < num_alloc) num_alloc = num_readings + 1;
  reading_size = 7 * sizeof(double);
  buffer_size = num_alloc * reading_size;
  buffer = (unsigned char *) malloc(buffer_size);
  if (!buffer) {
  	printf("\nError allocating buffer of size %d in IMUReadings::Read\n",
	       buffer_size);
    exit(0);
  }
  reserve(num_readings); /* Allocate vector space */
  
  // Read the data
  num_bytes_left = 0;
  while (!feof(fd) && size() < num_readings) {
    // Read data into the buffer
    num_bytes = fread(buffer + num_bytes_left, sizeof(unsigned char),
                      buffer_size - num_bytes_left, fd) + num_bytes_left;
    if (num_bytes == 0) {
      printf("\nNo bytes could be read, but still %d IMU readings to go\n",
	         num_readings - size());
      printf("ferror returns %d\n", ferror(fd));
      perror("perror output:");
      exit(0);
    }
    index = 0;
    // Extract IMU readings from the buffer
    while (size() < num_readings &&
           (feof(fd) || index < num_bytes - reading_size)) {
      memcpy((void *) reading, (const void *) (buffer + index),
             reading_size);
      index += reading_size;
      push_back(IMUReading(reading[0], reading[1], reading[2], reading[3],
	                       reading[4], reading[5], reading[6]));
    }
    // Transfer the remainder of the buffer to the begin of the buffer
    num_bytes_left = num_bytes - index;
    if (num_bytes_left > 0)
      memmove((void *) buffer, (const void *) (buffer + index), num_bytes_left);
  }
  free(buffer);
  return(1);
}

/*
--------------------------------------------------------------------------------
                          Write IMU readings to file
--------------------------------------------------------------------------------
*/

int IMUReadings::Write(const char *filename)
{
  if (imu_file) free(imu_file);
  imu_file = (char *) malloc(strlen(filename) + 1);
  strcpy(imu_file, filename);
  return(Write());
}

int IMUReadings::Write() const
{
  IMUReadings::const_iterator reading;
  int     file_id, num_bytes, num_readings, *endings, i, *ending, num_alloc,
          reading_size;
  FILE    *fd;
  char    *reserved_space, *command;
  double  *readings, *readings_ptr;

  // Open the IMU file
#ifdef windows
  if ((fd = fopen(imu_file, "wb")) == NULL) {
#else
  if ((fd = fopen(imu_file, "w")) == NULL) {
#endif
    fprintf(stderr, "Could not open IMU file %s for writing\n", imu_file);
    return(0);
  }

  // Write the header information
  file_id = LASER_IMU;
  fwrite((const void *) &file_id, sizeof(int), 1, fd);  num_bytes = sizeof(int);
  num_readings = size();
  fwrite((const void *) &num_readings, sizeof(int), 1, fd);
  num_bytes += sizeof(int);

  // Reserve the rest of the 1024 byte header for future use
  num_bytes = 1024 - num_bytes;
  reserved_space = (char *) malloc(num_bytes);
  fwrite((const void *) reserved_space, sizeof(char), num_bytes, fd);
  free(reserved_space);

  // Memory allocation
  num_alloc = 10000;
  if (num_readings < num_alloc) num_alloc = num_readings;
  reading_size = 7 * sizeof(double);
  readings = (double *) malloc(reading_size * num_alloc);
  if (readings == NULL) {
    fprintf(stderr, "Error allocating memory for IMU readings\n");
    return(0);
  }

  // Write the data
  for (reading=begin(), i=0, readings_ptr=readings;
       reading!=end();
       reading++, i++, readings_ptr+=7) {
    if (i == num_alloc) {
      fwrite((const void *) readings, reading_size, num_alloc, fd);
      i = 0;
      readings_ptr = readings;
    }
    *readings_ptr = reading->Time();
    *(readings_ptr+1) = reading->AngularVelocities().X();
    *(readings_ptr+2) = reading->AngularVelocities().Y();
    *(readings_ptr+3) = reading->AngularVelocities().Z();
    *(readings_ptr+4) = reading->Accelerations().X();
    *(readings_ptr+5) = reading->Accelerations().Y();
    *(readings_ptr+6) = reading->Accelerations().Z();
  }
  fwrite((const void *) readings, reading_size, i, fd);
  free(readings);

  // Close the file
  fclose(fd);

  return(1);
}

/*
--------------------------------------------------------------------------------
                          Set the IMU file name
--------------------------------------------------------------------------------
*/

void IMUReadings::SetIMUFile(const char *filename)
{
  StringCopy(&imu_file, filename);
}

/*
--------------------------------------------------------------------------------
                       Read the IMU readings meta data
--------------------------------------------------------------------------------
*/
int IMUReadings::ReadMetaData(const char *filename)
{
  SetMetaDataFile(filename);
  ReadMetaData();
}


int IMUReadings::ReadMetaData()
{
  FILE *fd;
  char *buffer, *keyword, *line;
  int  keyword_length, success;

  // Open the file
  fd = fopen(meta_file, "r");
  if (fd == NULL) {
    fprintf(stderr, "Error opening IMU meta data file %s\n", meta_file);
    return(0);
  }

  // Verify that the first keyword is "imureadings"

  buffer = (char *) malloc(MAXCHARS);
  do {
    line = fgets(buffer, MAXCHARS, fd);
  } while (line && Is_Comment(line));
  if (line == NULL) {
    fprintf(stderr, "Error reading IMU meta data file %s\n", meta_file);
    fclose(fd);
    free(buffer);
    return(0);
  }
  keyword = BNF_KeyWord(line, &keyword_length);
  if (strncmp(keyword, "imureadings", MAX(keyword_length, 11))) {
    fprintf(stderr, "Error: File %s is not an IMU meta data file.\n",
            meta_file);
    fprintf(stderr, "       First keyword is %s, not imureadings.\n",
	        keyword);
  }

  success = ReadMetaData(fd);
  fclose(fd);
  free(buffer);
  return success;
}

int IMUReadings::ReadMetaData(FILE *fd)
{
  char *line, *keyword, *buffer;
  int  keyword_length;
  
  // Initialise all meta data
  ReInitialise();

  // Process all lines
  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
	
        if (!strncmp(keyword, "name", MAX(keyword_length, 4)))
          name = BNF_String(line);

        else if (!strncmp(keyword, "metadata", MAX(keyword_length, 8)))
          SetMetaDataFile(BNF_String(line));

        else if (!strncmp(keyword, "imudata", MAX(keyword_length, 7)))
          imu_file = BNF_String(line);

        else if (!strncmp(keyword, "start_time", MAX(keyword_length, 10)))
          start_time = BNF_Double(line);

        else if (!strncmp(keyword, "end_time", MAX(keyword_length, 8)))
          end_time = BNF_Double(line);

        else if (!strncmp(keyword,"endimureadings",MAX(keyword_length,14))) {
          free(buffer);
          return(1);
        }
        
        else {
          keyword[keyword_length] = 0;
          fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
        }
      }
    }
  }
  fprintf(stderr, "Error: Did not find keyword endimureadings.\n");
  fclose(fd);
  return(0);
}


/*
--------------------------------------------------------------------------------
                   Write the IMU meta data to file
--------------------------------------------------------------------------------
*/

int IMUReadings::WriteMetaData() const
{
  return(WriteMetaData(meta_file));
}

int IMUReadings::WriteMetaData(const char *filename) const
{
  FILE       *fd;
  int        success;

  // Open the file
  fd = fopen(filename, "w");
  if (fd == NULL) {
    fprintf(stderr, "Error opening IMU meta data file %s\n", filename);
    return(0);
  }

  // Write the meta data
  success =  WriteMetaData(fd);
  
  // Close file and return success code
  fclose(fd);
  return success;
}

int IMUReadings::WriteMetaData(FILE *fd) const
{
  int        indent;

  indent = 0;
  BNF_Write_String(fd, "imureadings", indent, NULL);  indent += 2;
  if (name) BNF_Write_String(fd, "name", indent, name);
  if (meta_file) BNF_Write_String(fd, "metadata", indent, meta_file);
  if (imu_file) BNF_Write_String(fd, "imudata", indent, imu_file);
  if (start_time != -1.0)
    BNF_Write_Double(fd, "start_time", indent, start_time, "%.6f");
  if (end_time != -1.0)
    BNF_Write_Double(fd, "end_time", indent, end_time, "%.6f");
  indent -= 2;
  BNF_Write_String(fd, "endimureadings", indent, NULL);
  return(1);
}

/*
--------------------------------------------------------------------------------
                          Derive name of the sub unit
--------------------------------------------------------------------------------
*/

char *IMUReadings::DeriveName(const char *parentname, IMUReadings *first_part)
{
  int  part_number, i;
  char *ch;

  if (name) free(name);
  name = (char *) malloc(strlen(parentname) + 8);
  part_number = ((int) this - (int) first_part) / sizeof(IMUReadings);
  sprintf(name, "%s_%6d", parentname, part_number);
  for (i=strlen(parentname), ch=name+i; i<strlen(name); i++, ch++)
    if (*ch == ' ') *ch = '0';
  return(name);
}

/*
--------------------------------------------------------------------------------
                    Derive IMU data and meta data file names
--------------------------------------------------------------------------------
*/

char *IMUReadings::DeriveIMUDataFileName(const char *directory)
{
  if (!name) return(NULL);  /* We need a name */
  if (imu_file) free(imu_file);
  imu_file = ComposeFileName(directory, name, ".imu");
  return(imu_file);
}

char *IMUReadings::DeriveMetaDataFileName(const char *directory)
{
  return(LaserMetaFile::DeriveMetaDataFileName(directory, ".imupart"));
}

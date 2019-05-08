
/*
    Copyright 2010 University of Twente and Delft University of Technology
 
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


/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : Feb 2006
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Partition3D of any container (LaserPoints, vector<Vector3D> stxxl::vector<LaserPoint>)
*				The value_type of the container must support [0],[1] and [2].
*
*--------------------------------------------------------------------*/
#ifndef __PARTITION___3D__H__
#define __PARTITION___3D__H__

///Partition 3D of a container into cubic boxes.
#include"PartitionBox.h"
#include "PartitionSelectors.h"
#include<map>
using std::map;

template <class T>	
class Partition3D : public std::map<PartitionBox<T>,SelectorVector<T>,PartitionBoxCompare<T> >
{
public:
	//typedef hash_map<PartitionBox<T>,PartitionBoxData,HashPartitionBox<T>,HashPartitionBoxEqual<T> > PartitionMap;
	typedef PartitionBox<T> PartitionBoxType;
	typedef SelectorVector<T> SelectorVectorType;
	typedef  std::map<PartitionBoxType,SelectorVectorType> PartitionMap;
	typedef std::set<PartitionBoxType,PartitionBoxCompare<T> > PartitionBoxSet;
	typedef std::vector<PartitionBoxType> PartitionBoxVector;
	
	Partition3D(double xC,double yC,double zC)
	{
		UpdateConstants(xC,yC,zC);
	}
	
	Partition3D(double stepSize)
	{
		UpdateConstants(stepSize,stepSize,stepSize);	
	}
	
	template<class TLaserPoints>
	Partition3D(const TLaserPoints& pts,double stepSize)
	{
		UpdateConstants(stepSize,stepSize,stepSize);
		Update(pts);	
	}
	
	int64 Count()const
	{
		int64 total = 0;
		for(PartitionMap::const_iterator iter = begin();iter!=end();iter++)
			total+= (iter->second)[0]->Size();
		return total;
	}
	
	int64 Count(const PartitionBoxVector& vec)const
	{
		return this->Count(vec.begin(),vec.end());
	}
	
	template<class Iter>
	int64 Count(Iter i1, Iter i2)const
	{
		int64 total = 0;
		for(Iter it=i1; it!=i2; it++)
		{
			PartitionMap::const_iterator iter = this->find(*it);
			if(iter!=this->end())
				total+= iter->second[0]->Size();	
		}
		return total;
	}
	
	void UpdateConstants(double xScale, double yScale, double zScale)
	{
		xFactor = 1.00/xScale;
		yFactor = 1.00/yScale;
		zFactor = 1.00/zScale;
	}
	
	template<class TLaserPoint>
	PartitionBox<T> Point2Box(const TLaserPoint& pt)const
	{
		return PartitionBox<T>((T)((pt[0])*xFactor),(T)((pt[1])*yFactor),
						(T)((pt[2])*zFactor));
	}
	
	PartitionBox<T> Point2Box(double x, double y, double z)const
	{
		return PartitionBox<T>((T)(x)*xFactor,(T)(y)*yFactor,(T)(z)*zFactor);
	}
	
	LaserPoint Box2Point(const PartitionBox<T> b,int count=0)const
	{
		return LaserPoint(b.x/xFactor,b.y/yFactor,b.z/zFactor,count);
	}
	
	PartitionBoxVector Get27Neighbors(const PartitionBoxType& queryBox)const
	{
		PartitionBoxVector resultVec;
		for(int x=-1;x<=1;x++)
		for(int y=-1;y<=1;y++)
		for(int z=-1;z<=1;z++)
		{
			PartitionBoxType box(queryBox.x+x, queryBox.y+y, queryBox.z+z);
			
			if(BoxExists(box))
				resultVec.push_back(box);
		}
		return resultVec;
	}
	
	PartitionBoxVector Get27Neighbors(const PartitionBoxVector& queryBoxes)const
	{
		return Get27Neighbors(queryBoxes.begin(),queryBoxes.end());
	}
	
	template<class Iter>
	PartitionBoxVector Get27Neighbors(Iter it1, Iter it2)const
	{
		PartitionBoxSet boxSet;
		
		for(Iter k=it1; k!=it2; k++)
		{
			PartitionBoxVector vec = Get27Neighbors(*k);
			for(int i=0;i<vec.size();i++)
				boxSet.insert(vec[i]);
		}
		PartitionBoxVector vec(boxSet.size());
		std::copy(boxSet.begin(),boxSet.end(),vec.begin());
		
		return vec;
	}
		
	
	bool BoxExists(const PartitionBoxType& box)const
	{
		return this->find(box)!=this->end();
	}
	
	template<class TLaserPoints>
	int Update(const TLaserPoints& laserPoints)
	{
		//clear the map.
		this->clear();
		
		Add(laserPoints);
			
		return 0;
	}
	
	template<class TLaserPoints>
	int Add(const TLaserPoints& laserPoints)
	{
		for(int i=0;i<laserPoints.size();i++)
		{
			PartitionBox<T> box = Point2Box(laserPoints[i]);
			
			//If the key doesn't exist insert new node
			if(find(box) == this->end())
			{
				(*this)[box].push_back(new IndicesSelector<T>()	);
			}
			((IndicesSelector<T>*)((*this)[box][0]))->push_back(i);
		}
	}

	//Prints some statistics to console.
	ostream& Print(ostream& os=std::cerr)const
	{
		os<<"\n\nPartition3D: "<<this<<"\n";
		os<<"Box scale are: ("<<1.00/xFactor<<", "<<1.00/yFactor<<", "<<1.00/zFactor<<")"<<endl;
		os<<"Map entries are: "<<this->size()<<endl;
		os<<"Indices size is: "<<this->Count()<<endl;
		
		int count = 0;
		PartitionMap::const_iterator iter;
		int stepSize = std::max((int)this->size()/10,1);
		for(iter = begin();iter!= end();iter++,count++)
		{
			if(count%stepSize == 0)
			{
				os<<"Box of type "<< (iter->second)[0]->Name()
					<<" with count: "<<(iter->second)[0]->Size()<<"   ";
			}
		}
		os<<endl;
		
		return os;
	}
	
	//Write new stxxl file, so that all selections are changed to range selection.
	//Note that after this operation the partition is updated for the destination vector.
	//DONOT try to use it for the src object, as it would give meaningless results.
	template<class SrcType, class DestType>
	bool WriteArrangedStxxl(const SrcType& src, DestType& dest)
	{
		dest.resize(0);
		dest.reserve(src.size());
		int64 pointer = 0;
		for(PartitionMap::iterator iter = begin(); iter!= end(); iter++)
		{
			//Write the current selection to destination vector.
			iter->second.AddSelection(src,dest);
			
			//Delete the old selection.
			iter->second.Destroy();
			iter->second.resize(1);
						
			//Create a new range selection. As the destination vector has all elements in 
			//right order we donot need to keep all these indices.
			//Just store start and end index.
			iter->second[0] = new RangeSelector<SelectorVector<T>::IndicesType>(pointer,dest.size());
			
			//update the pointer to point to the new beginning.
			pointer = dest.size();
		}
		return true;
	}
	
	std::ostream& Save(std::ostream& os)const
	{
		const char spacing[] = " ";	
		os<<"Partition3D"<<spacing;
		os<<setprecision(10);
		
		//Factors.
		os<<xFactor<<spacing;
		os<<yFactor<<spacing;
		os<<zFactor<<spacing;
		
		//Number of map entries.
		os<<this->size();
		
		//All map entries.
		for(PartitionMap::const_iterator iter = begin(); iter!= end(); iter++)
		{
			iter->first.Save(os);
			iter->second.Save(os);
		}
		os<<endl;
		return os;
	}
	
	std::istream& Load(std::istream& is)
	{
		std::string header;
		
		is>>header;
		
		if(header!=std::string("Partition3D"))
		{
			cerr<<"ERROR: Invalid header in Partition3D::Load\n";
			return is;
		}
		
		//Factors.
		is >> xFactor >> yFactor  >> zFactor;
		
		//Number of map entries.
		int entryCount;
		is >> entryCount;
		
		//Clear previous entries.
		this->clear();
		
		//Read All map entries.
		for(int i=0; i<entryCount; i++)
		{
			PartitionBoxType box;
			box.Load(is);
			
			(*this)[box].Load(is);		
		}
		//Now we are done.
		return is;
	}
	
	void Save(std::string fileName)const
	{
		std::ofstream file(fileName.c_str());
		
		if(file.good())
		{
			this->Save(file);
			file.close();
		}
	}
	
	void Load(std::string fileName)
	{
		std::ifstream file(fileName.c_str());
		
		if(file.good())
		{
			this->Load(file);
			file.close();
		}
	}
	
	template<class SrcType, class DestType, class Iter>
	DestType& AddSelection(const SrcType& src, DestType& dest, Iter it1, Iter it2)
	{
		for(Iter it=it1; it!=it2; it++)
		{
			if(BoxExists(*it))
			{
				(*this)[*it].AddSelection(src,dest);
			}		
		}
		return dest;
	}
	
	template<class SrcType, class DestType>
	DestType& AddSelection(const SrcType& src, DestType& dest, const PartitionBoxVector& vec)
	{
		return AddSelection(src, dest, vec.begin(), vec.end());
	}
	
	template<class SrcType, class DestType>
	DestType& Normals(const SrcType& src, DestType& normals, int kNN)
	{
		normals.resize(src.size());
		
		ProgressDisplay<int>& progress__ = *(ProgressDisplayManager::GetProgressObject());
		ProgressDisplay<int> progress;
		progress.Initialize(this->size(),"Progress");
		progress.Print();
		int counter = 0;
		
		//Walk over All map entries.
		for(PartitionMap::const_iterator iter = begin(); iter!= end(); iter++, counter++)
		{
			std::vector<int64> indices = iter->second.SelectedIndices();
		
			LaserPoints currentNeighboringPts;
			AddSelection(src, currentNeighboringPts, Get27Neighbors(iter->first));
			
			if(currentNeighboringPts.size()>=kNN)
			{

				KNNFinder<LaserPoint> finder(currentNeighboringPts);

				for(int i=0;i<indices.size();i++)
				{
					int64 currentIndex = indices[i];

					std::vector<int> kNNIndices = finder.FindIndices(src[currentIndex],kNN);
					normals[currentIndex] = currentNeighboringPts.Select(kNNIndices).Normal();
				}
			}
			else
			{
				//We donot have enough points so just write default normal.
				for(int i=0;i<indices.size();i++)
				{
					int64 currentIndex = indices[i];
					normals[currentIndex] = Vector3D(1,0,0);
				}		
			}
			
			progress.Step("Calculating normals",counter);
			
		}
		progress.End("Calculating normals");
		return normals;
	}
private:
	double xFactor,yFactor,zFactor;
};
//// --- Space partition ends ---
#endif //__PARTITION3D__H__
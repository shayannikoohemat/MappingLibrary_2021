
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
*   Purpose   : Selectors for storing information about selection 
*               (IndicesRange, IndicesVector and SelectorVector etc.)
*
*--------------------------------------------------------------------*/
#ifndef __PARTITION___SELECTORS__H__
#define __PARTITION___SELECTORS__H__
#include "MyTypeDefinitions.h"
#include <vector>
using namespace std;

//**********************************************************************************
//	Selector classes.                                                                                    
//**********************************************************************************

template<class T>
class BaseSelector
{
public:
///Define an enum to keep the type of the selector.
///We would have preffered to use virtual functions but C++ doesn't allow template members to be
///virtual and some functions need to be templatized so that they can transparently work with 
///both std::vector and stxxl::vector.
enum SelectorType{BaseSelectorType,RangeSelectorType,IndicesSelectorType,SelectorVectorType};

	///Constructor.
	BaseSelector()
	:selectorType(BaseSelectorType)
	{
	
	}
	///Return the type of this selector.
	SelectorType Type()const
	{
		return selectorType;
	}
	
	///Return the selected indices.
	virtual std::vector<int64> SelectedIndices()const
	{
		return std::vector<int64>();	
	};
	
	///Return the size of selection.
	virtual int64 Size()const
	{
		return 0;
	}
	
	///Gives the name of the selector.
	virtual std::string Name()const
	{
		return "BaseSelector";
	}
	
	///Save to an output stream.
	virtual std::ostream& Save(std::ostream& os)const
	{
		//Noting to save here.
		return os;
	}

	///Load from an input stream
	virtual std::istream& Load(std::istream& os)
	{
		//Nothing to load here.
		return os;
	}
	
	///Templatized add selection function. Works on the concept of src and sink
	///both of which have to support [] and push_back operators.
	template<typename PointSrc, typename PointSink>
	PointSink& AddSelection(const PointSrc& src, PointSink& sink)
	{
		//Not much to do for base selector, just return the sink.
		return sink;	
	}
protected:
	SelectorType selectorType;	
};

///This selector keep a range of indices for begin to end(non-inclusive)
template<class T>
class RangeSelector:public BaseSelector<T>
{
public:
///Constructor.
RangeSelector(T _begin=-1, T _end=-1)
:begin(_begin),end(_end)
{
	selectorType = RangeSelectorType;
};

///Gives the name of the selector.
std::string Name()const
{
	return "RangeSelector";
}

///Save to an output stream.
virtual std::ostream& Save(std::ostream& os)const
{
	os<<Name()<<" "<<begin<<" "<<end<<" ";
	return os;
}

///Load from an input stream
virtual std::istream& Load(std::istream& is)
{
	is>>begin;
	is>>end;
	return is;
}

///Return the size of selection.
virtual int64 Size()const
{
	if(begin>=0 && end>=0 && end>begin)
		return (end-begin);
	return 0;
}
	

std::vector<int64> SelectedIndices()const
{
	std::vector<int64> results;
	//cerr<<"RangeSelector::SelectedIndices() \n";
	
	for(int64 i=begin; i<end; i++)
		if(i>=0)
			results.push_back(i);
	
	return results;
}

template<typename PointSrc, typename PointSink>
PointSink& AddSelection(const PointSrc& src, PointSink& sink)
{
	//cerr<<"RangeSelector::AddSelection() \n";
	
	for(int64 i=begin; i<end; i++)
		if(i>=0 && i<src.size())
			sink.push_back(src[i]);

	return sink;	
}

protected:
//Beginning and ending index. similar to begin(), end() of std::vector.
T begin, end;

};


///This selector keeps indices in a separate vector.
template<class T>
class IndicesSelector:public BaseSelector<T>, public std::vector<T>
{
public:
///Constructor.
IndicesSelector()
{
	selectorType = IndicesSelectorType;
};

///Gives the name of the selector.
std::string Name()const
{
	return "IndicesSelector";
}

///Save to an output stream.
virtual std::ostream& Save(std::ostream& os)const
{
	os<<Name()<<" "<<size()<<" ";
	
	for(int i=0;i<size();i++)
		os<<(*this)[i]<<" ";	
	return os;
}

///Return the size of selection.
virtual int64 Size()const
{
	return ((std::vector<T>*)(this))->size();
}

///Load from an input stream
virtual std::istream& Load(std::istream& is)
{
	T temp;
	
	//Read in the size.
	is>>temp;
	this->resize(temp);
	
	for(int i=0;i<size();i++)
		is>>((*this)[i]);
	return is;
}


std::vector<int64> SelectedIndices()const
{
	std::vector<int64> results;
	//cerr<<"IndicesSelector::SelectedIndices() \n";
	
	results.resize(this->size());
	std::copy(this->begin(),this->end(),results.begin());
	
	return results;
}

template<typename PointSrc, typename PointSink>
PointSink& AddSelection(const PointSrc& src, PointSink& sink)
{
	//cerr<<"IndicesSelector::AddSelection() \n";
	for(int i=0;i<this->size();i++)
		sink.push_back(src[(*this)[i]]);

	return sink;	
}

};


///This is a vector of selectors.
template<typename T>
class SelectorVector: public std::vector<BaseSelector<T>* >, public BaseSelector<T>
{
public:
///constructor (also Default)
typedef T IndicesType;
typedef  BaseSelector<T> BaseType;
typedef  std::vector<BaseSelector<T>* > VectorType;

SelectorVector()
{
	selectorType = SelectorVectorType;	
}

///Gives the name of the selector.
std::string Name()const
{
	return "SelectorVector";
}

///Save to an output stream.
std::ostream& Save(std::ostream& os)const
{
	os<<Name()<<" "<<size()<<" ";
	for(int i=0;i<size();i++)
		(*this)[i]->Save(os);
		
	return os;
}

std::vector<int64> SelectedIndices()const
{
	std::vector<int64> results;
	
	for(int i=0;i<size();i++)
		Append(results, (*this)[i]->SelectedIndices());
		
	return results;
}

///Load from an input stream
std::istream& Load(std::istream& is)
{
	//Destory the old vector.
	this->Destroy();
	
	string name;
	T temp;
	
	is>>name;
	
	if(name != "SelectorVector")
	{
		cerr<<"ERR: SelectorVector::Load() -> Unrecognized name "<<name<<" instead of wanted SelectorVector \n";
		return is;
	}
	
	is>>temp;

	
	BaseType* obj = NULL;	
	for(int i=0;i<temp;i++)
	{
		is>>name;
		
		if(name == "BaseSelector")
		{
			obj = new BaseSelector<T>();
		}
		else if(name == "RangeSelector")
		{
			obj = new RangeSelector<T>();
		}
		else if(name == "IndicesSelector")
		{
			obj = new IndicesSelector<T>();
		}
		else if(name == "SelectorVector")
		{
			obj = new SelectorVector<T>();
			//Put it back on the stream, as we will read it again.
			for(int i=0;i<15;i++)
				is.unget();
		}
		else
		{
			cerr<<"ERR: SelectorVector::Load() -> Unrecognized name "<<name<<"\n";
		}
		
		//Load and add to vector.
		if(obj)
		{
			obj->Load(is);
			this->push_back(obj);
		}
	}
}

///Destroy this vector.
void Destroy()
{
	for(int i=0;i<size();i++)
		delete((*this)[i]);
	clear();
}

///Destroy the pointers before exit.
~SelectorVector()
{
	this->Destroy();	
}


template<typename PointSrc, typename PointSink>
PointSink& AddSelection(const PointSrc& src, PointSink& sink)const
{
	//cerr<<"SelectorVector::AddSelection() \n";
	
	for(int i=0;i<size();i++)
	{
		BaseType* sel = (*this)[i];
		
		if(sel->Type()==BaseType::RangeSelectorType)
		{
			((RangeSelector<T>*)sel)->AddSelection(src,sink);
		}
		else if(sel->Type()==BaseType::IndicesSelectorType)
		{
			((IndicesSelector<T>*)sel)->AddSelection(src,sink);
		}
		else if(sel->Type()==BaseType::SelectorVectorType)
		{
			if(sel==this)
			{
				cerr<<"ERR: infinite recursion in SelectorVector::AddSelection	\n";
				return sink;
			}
			else
				(*((SelectorVector<T>*)sel)).AddSelection<PointSrc,PointSink>(src,sink);
		}
		else if (sel->Type()==BaseType::BaseSelectorType)
		{
			//Nothing to do here
		}
		else
		{
			cerr<<"Uknown type "<<sel->Type()<<" in SelectorVector::AddSelection()\n";
		}
	}

	return sink;	
}


};

#endif //__PARTITION_SELECTORS__H__
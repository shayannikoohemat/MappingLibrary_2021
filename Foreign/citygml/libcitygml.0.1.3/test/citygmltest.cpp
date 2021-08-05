#include <iostream>
#include <fstream>
#include <time.h> 
#include <algorithm>
#include "citygml.h"

void analyzeObject( const citygml::CityObject*, unsigned int );

void usage() 
{
	std::cout << "Usage: citygmltest [-log] [-filter <mask>] <filename>" << std::endl;
	std::cout << " Options:" << std::endl;
	std::cout << "  -log            Print some informations during parsing" << std::endl;
	std::cout << "  -filter <mask>  CityGML objects to parse (default:All)" << std::endl
		<< "                  The mask is composed of:" << std::endl
		<< "                   GenericCityObject, Building, Room," << std::endl
		<< "                   BuildingInstallation, BuildingFurniture, Door, Window, " << std::endl
		<< "                   CityFurniture, Track, Road, Railway, Square, PlantCover," << std::endl
		<< "                   SolitaryVegetationObject, WaterBody, TINRelief, LandUse," << std::endl
		<< "                   Tunnel, Bridge, BridgeConstructionElement," << std::endl
		<< "                   BridgeInstallation, BridgePart,  All" << std::endl
		<< "                  and seperators |,&,~." << std::endl
		<< "                  Examples:" << std::endl
		<< "                  \"All&~Track&~Room\" to parse everything but tracks & rooms" << std::endl
		<< "                  \"Road&Railway\" to parse only roads & railways" << std::endl;
	exit(-1);
}
int main( int argc, char **argv )
{
	if ( argc < 2 ) usage();

	int fargc = 1;

	bool log = false;
	std::string filter = "All";

	for ( int i = 1; i < argc; i++ ) 
	{
		std::string param = std::string( argv[i] );
		std::transform( param.begin(), param.end(), param.begin(), tolower );
		if ( param == "-log" ) { log = true; fargc = i+1; }
		if ( param == "-filter" ) { if ( i == argc - 1 ) usage(); filter = argv[i+1]; i++; fargc = i+1; }
	}
	if ( argc - fargc < 1 ) usage();

	std::cout << "Parsing CityGML file " << argv[fargc] << " using libcitygml v." << LIBCITYGML_VERSIONSTR << "..." << std::endl;

	time_t start;
	time( &start );

	citygml::ParserParams params;
	params.objectsMask = filter;

#if 0
	std::ifstream file;
	file.open( argv[fargc], std::ifstream::in );
	citygml::CityModel *city = citygml::load( file, filter );
#else
	citygml::CityModel *city = citygml::load( argv[fargc], params );
#endif

	time_t end;
	time( &end );

	if ( !city ) return NULL;

	std::cout << "Done in " << difftime( end, start ) << " seconds." << std::endl; 

	std::cout << city->size() << " city objects read." << std::endl;

	std::cout << "Analyzing the city objects..." << std::endl;


	const citygml::CityObjectsMap& cityObjectsMap = city->getCityObjectsMap();

	citygml::CityObjectsMap::const_iterator it = cityObjectsMap.begin();

	for ( ; it != cityObjectsMap.end(); it++ )
	{
		const citygml::CityObjects& v = it->second;

		std::cout << ( log ? " Analyzing " : " Found " ) << v.size() << " " << citygml::getCityObjectsClassName( it->first ) << ( ( v.size() > 1 ) ? "s" : "" ) << "..." << std::endl;

		if ( log ) 
		{
			for ( unsigned int i = 0; i < v.size(); i++ )
			{
				std::cout << "  + found object " << v[i]->getId();
				if ( v[i]->getChildCount() > 0 ) std::cout << " with " << v[i]->getChildCount() << " children";
				std::cout << " with " << v[i]->size() << " geometr" << ( ( v[i]->size() > 1 ) ? "ies" : "y" );
				std::cout << std::endl;
			}
		}
	}

	if ( log ) 
	{
		std::cout << std::endl << "Objects hierarchy:" << std::endl;
		const citygml::CityObjects& roots = city->getCityObjectsRoots();

		for ( unsigned int i = 0; i < roots.size(); i++ )

			analyzeObject( roots[ i ], 2 );
	}

	std::cout << "Done." << std::endl;

	return 0;
}

void analyzeObject( const citygml::CityObject* object, unsigned int indent )
{
	for ( unsigned int i = 0; i < indent; i++ ) std::cout << " ";
		std::cout << "Object " << citygml::getCityObjectsClassName( object->getType() ) << ": " << object->getId() << std::endl;

	for ( unsigned int i = 0; i < object->getChildCount(); i++ )
		analyzeObject( object->getChild(i), indent+1 );
}
///////////////////////////////////////////////////////////////////////////////
// OSG plugin for reading OGC CityGML v0.3 - v1.0 format using libcitygml
// http://code.google.com/p/libcitygml
// Copyright(c) 2010 Joachim Pouderoux, BRGM
//////////////////////////////////////////////////////////////////////////

#include <osg/Notify>
#include <osg/Node>
#include <osg/Geometry>
#include <osg/ProxyNode>
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/Material>
#include <osg/Texture2D>
#include <osg/TexGen>
#include <osg/TexMat>
#include <osg/ShapeDrawable>
#include <osg/Shape>
#include <osg/LightModel>
#include <osg/Point>

#include <osgText/Font>
#include <osgText/Text>

#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include "citygml.h"

#include <algorithm>
#include <cctype> 


class ReaderWriterCityGML : public osgDB::ReaderWriter
{
public:
	ReaderWriterCityGML( void )
	{ 
		supportsExtension( "citygml", "CityGML format" );

		supportsOption( "names", "Add the name of the city objects on top of them" );
		supportsOption( "mask", "Set the objects mask" );
		supportsOption( "minLOD", "Minimum LOD level to fetch" );
		supportsOption( "maxLOD", "Maximum LOD level to fetch" );
		supportsOption( "optimize", "Optimize the geometries & polygons of the CityGML model to reduce the number of instanced objects" );
		supportsOption( "pruneEmptyObjects", "Prune empty objects (ie. without -supported- geometry)" );
	}

	virtual const char* className( void ) const { return "CityGML Reader"; }

	virtual ReadResult readNode( const std::string&, const osgDB::ReaderWriter::Options* ) const;

private:
	class Settings 
	{
	public:
		Settings( void ) : _printNames( false ) {}

		void parseOptions( const osgDB::ReaderWriter::Options* options)
		{
			if ( !options ) return;
			std::istringstream iss( options->getOptionString() );
			std::string currentOption;
			while ( iss >> currentOption )
			{
				std::transform( currentOption.begin(), currentOption.end(), currentOption.begin(), tolower );
				if ( currentOption == "names" ) _printNames = true;
				else if ( currentOption == "mask" ) iss >> _params.objectsMask;
				else if ( currentOption == "minlod" ) iss >> _params.minLOD;
				else if ( currentOption == "maxlod" ) iss >> _params.maxLOD;
				else if ( currentOption == "optimize" ) _params.optimize = true;
				else if ( currentOption == "pruneemptyobjects" ) _params.pruneEmptyObjects = true;		
			}
		}

	public:
		bool _printNames;
		citygml::ParserParams _params;
		std::map< std::string, osg::Texture2D* > _textureMap;
	};

private:
	bool createCityObject( const citygml::CityObject*, Settings&, osg::Group* ) const;
};

// Register with Registry to instantiate the above reader/writer.
REGISTER_OSGPLUGIN( citygml, ReaderWriterCityGML )

// Read CityGML file using libcitygml and generate the OSG scenegraph
osgDB::ReaderWriter::ReadResult ReaderWriterCityGML::readNode( const std::string& file, const osgDB::ReaderWriter::Options* options ) const
{
	std::string ext = osgDB::getLowerCaseFileExtension( file );
	if ( !acceptsExtension( ext ) ) return ReadResult::FILE_NOT_HANDLED;

	// try to open the file as is
	std::string fileName = osgDB::findDataFile( file, options );
	if ( fileName.empty() )
	{
		// not found, so remove the .citygml extension file
		std::string fname = osgDB::getNameLessExtension( file );
		fileName = osgDB::findDataFile( fname, options );
		if ( fileName.empty() ) return ReadResult::FILE_NOT_FOUND;
	}

	Settings settings;
	settings.parseOptions( options );

	osgDB::getDataFilePathList().push_front( osgDB::getFilePath( fileName ) );

	// Redirect both std::cout & std::cerr (used by libcitygml) to osg::notify stream
	std::streambuf* coutsb = std::cout.rdbuf( osg::notify(osg::NOTICE).rdbuf() );
	std::streambuf* cerrsb = std::cerr.rdbuf( osg::notify(osg::NOTICE).rdbuf() );

	osg::notify(osg::NOTICE) << "Parsing CityGML file " << fileName << "..." << std::endl;

	citygml::CityModel *city = citygml::load( fileName, settings._params );

	if ( !city ) return NULL;

	osg::notify(osg::NOTICE) << city->size() << " city objects read." << std::endl;

	osg::notify(osg::NOTICE) << "Creation of the OSG city objects' geometry..." << std::endl;

	osg::Group* root = new osg::Group();
	root->setName( fileName );

#define RECURSIVE_DUMP

#ifndef RECURSIVE_DUMP

	const citygml::CityObjectsMap& cityObjectsMap = city->getCityObjectsMap();

	citygml::CityObjectsMap::const_iterator it = cityObjectsMap.begin();

	for ( ; it != cityObjectsMap.end(); it++ )
	{
		const citygml::CityObjects& v = it->second;

		osg::notify(osg::NOTICE) << " Creation of " << v.size() << " " << citygml::getCityObjectsClassName( it->first ) << ( ( v.size() > 1 ) ? "s" : "" ) << "..." << std::endl;

		osg::Group* grp = new osg::Group;
		grp->setName( citygml::getCityObjectsClassName( it->first ) );
		root->addChild( grp );

		for ( unsigned int i = 0; i < v.size(); i++ )

			createCityObject( v[i], settings, grp ) )		
	}
#else

	const citygml::CityObjects& roots = city->getCityObjectsRoots();

	for ( unsigned int i = 0; i < roots.size(); i++ ) 

		createCityObject( roots[i], settings, root );
#endif

	osg::notify(osg::NOTICE) << "Done." << std::endl;

	delete city;

	// Restore cout/cerr streams
	std::cout.rdbuf( coutsb );
	std::cerr.rdbuf( cerrsb );

	return root;
}

bool ReaderWriterCityGML::createCityObject( const citygml::CityObject* object, Settings& settings, osg::Group* parent ) const
{
	// Skip objects without geometry
	if ( !object || !parent ) return false;

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->setName( object->getId() );

#ifdef RECURSIVE_DUMP
	osg::Group* grp = new osg::Group;
	grp->setName( object->getId() );
	grp->addChild( geode );
	parent->addChild( grp );
#else
	parent->addChild( geode )
#endif
	//osg::notify(osg::NOTICE) << "Creating object " << object->getId() << std::endl;

	// Get the default color for the whole city object
	osg::ref_ptr<osg::Vec4Array> shared_colors = new osg::Vec4Array;
	shared_colors->push_back( osg::Vec4( object->getDefaultColor().r, object->getDefaultColor().g, object->getDefaultColor().b, 1.f ) );

	osg::ref_ptr<osg::Vec4Array> roof_color = new osg::Vec4Array;
	roof_color->push_back( osg::Vec4( 0.9f, 0.1f, 0.1f, 1.0f ) );

	for ( unsigned int i = 0; i < object->size(); i++ ) 
	{
		const citygml::Geometry& geometry = *object->getGeometry( i );

		for ( unsigned int j = 0; j < geometry.size(); j++ ) 
		{
			const citygml::Polygon* p = geometry[j];
			if ( p->getIndices().size() == 0 ) continue;

			// Geometry management

			osg::Geometry* geom = new osg::Geometry();

			// Vertices
			osg::Vec3Array* vertices = new osg::Vec3Array;
			const std::vector<TVec3d>& vert = p->getVertices();
			vertices->reserve( vert.size() );
			for ( unsigned int k = 0; k < vert.size(); k++ )
			{
				osg::Vec3d pt( vert[k][0], vert[k][1], vert[k][2] );
				vertices->push_back( pt );
			}

			geom->setVertexArray( vertices );

			// Indices
			osg::DrawElementsUInt* indices = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLES, 0 );
			const std::vector<unsigned int>& ind = p->getIndices();
			indices->reserve( ind.size() );
			for ( unsigned int i = 0 ; i < ind.size() / 3; i++ )
			{
				indices->push_back( ind[ i * 3 + 0 ] );
				indices->push_back( ind[ i * 3 + 1 ] );
				indices->push_back( ind[ i * 3 + 2 ] );	
			}
			geom->addPrimitiveSet( indices );

			// Normals			
			osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
			const std::vector<TVec3f>& norm = p->getNormals();
			normals->reserve( norm.size() );
			for ( unsigned int k = 0; k < norm.size(); k++ )
				normals->push_back( osg::Vec3( norm[k].x, norm[k].y, norm[k].z ) );

			geom->setNormalArray( normals.get() );
			geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

			// Material management

			osg::ref_ptr<osg::StateSet> stateset = geom->getOrCreateStateSet();

			const citygml::Appearance *mat = p->getAppearance();

			bool colorset = false;

			if ( mat )
			{
				shared_colors->clear();
				shared_colors->push_back( osg::Vec4( 1.f, 1.f, 1.f, 1.f ) );

				if ( const citygml::Material* m = dynamic_cast<const citygml::Material*>( mat ) )
				{
#define TOVEC4(_t_) osg::Vec4( _t_.r, _t_.g, _t_.b, _t_.a ) 
					osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
					TVec4f diffuse( m->getDiffuse(), 0.f );
					TVec4f emissive( m->getEmissive(), 0.f );
					TVec4f specular( m->getSpecular(), 0.f );
					float ambient = m->getAmbientIntensity();

					osg::Material* material = new osg::Material;
					material->setColorMode( osg::Material::OFF );
					material->setDiffuse( osg::Material::FRONT_AND_BACK, TOVEC4( diffuse ) );
					material->setSpecular( osg::Material::FRONT_AND_BACK, TOVEC4( specular ) );
					material->setEmission( osg::Material::FRONT_AND_BACK, TOVEC4( emissive ) );					
					material->setShininess( osg::Material::FRONT_AND_BACK, m->getShininess() );					
					material->setAmbient( osg::Material::FRONT_AND_BACK, osg::Vec4( ambient, ambient, ambient, 1.0 ) );
					stateset->setAttributeAndModes( material, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON );
					stateset->setMode( GL_LIGHTING, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON );

					colorset = true;
				}
				else if ( const citygml::Texture* t = dynamic_cast<const citygml::Texture*>( mat ) ) 
				{
					const citygml::TexCoords& texCoords = p->getTexCoords();

					if ( texCoords.size() > 0 )
					{
						osg::Texture2D* texture = NULL;

						if ( settings._textureMap.find( t->getUrl() ) == settings._textureMap.end() )
						{
							// Load a new texture 
							osg::notify(osg::NOTICE) << "  Loading texture " << t->getUrl() << "..." << std::endl;

							if ( osg::Image* image = osgDB::readImageFile( t->getUrl() ) )
							{
								texture = new osg::Texture2D;
								texture->setImage( image );
								texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
								texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::NEAREST );
								texture->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT );
								texture->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );
								texture->setWrap( osg::Texture::WRAP_R, osg::Texture::REPEAT );
							}
							else
								osg::notify(osg::NOTICE) << "  Warning: Texture " << t->getUrl() << " not found!" << std::endl;

							settings._textureMap[ t->getUrl() ] = texture;
						}
						else
							texture = settings._textureMap[ t->getUrl() ];

						if ( texture )
						{
							osg::ref_ptr<osg::Vec2Array> tex = new osg::Vec2Array;
							tex->reserve( texCoords.size() );
							for ( unsigned int k = 0; k < texCoords.size(); k++ )
								tex->push_back( osg::Vec2( texCoords[k].x, texCoords[k].y ) );

							geom->setTexCoordArray( 0, tex );

							stateset->setTextureAttributeAndModes( 0, texture, osg::StateAttribute::ON );

							colorset = true;
						}
					}
					else 
						osg::notify(osg::NOTICE) << "  Warning: Texture coordinates not found for poly " << p->getId() << std::endl;
				}
			}

			// Color management

			geom->setColorArray( ( !colorset && geometry.getType() == citygml::GT_Roof ) ? roof_color.get() : shared_colors.get() );

			geom->setColorBinding( osg::Geometry::BIND_OVERALL );
#if 0
			// Set lighting model to two sided
			osg::ref_ptr< osg::LightModel > lightModel = new osg::LightModel;
			lightModel->setTwoSided( true );
			stateset->setAttributeAndModes( lightModel.get(), osg::StateAttribute::ON );
#endif
			// That's it!
			geode->addDrawable( geom );			
		}
	}

	if ( settings._printNames ) 
	{
		// Print the city object name on top of it
		geode->getBoundingBox().center();
		osg::ref_ptr<osgText::Text> text = new osgText::Text;
		text->setFont( "arial.ttf" );
		text->setCharacterSize( 2 );
		text->setBackdropType( osgText::Text::OUTLINE );
		text->setFontResolution( 64, 64 );
		text->setText( object->getId(), osgText::String::ENCODING_UTF8 );
		text->setCharacterSizeMode( osgText::TextBase::OBJECT_COORDS_WITH_MAXIMUM_SCREEN_SIZE_CAPPED_BY_FONT_HEIGHT );
		text->setAxisAlignment( osgText::TextBase::SCREEN );
		text->setAlignment( osgText::TextBase::CENTER_BOTTOM );
		text->setPosition( geode->getBoundingBox().center() + osg::Vec3( 0, 0, geode->getBoundingBox().radius() ) );
		text->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF );
		geode->addDrawable( text.get() );
	}

#ifdef RECURSIVE_DUMP
	for ( unsigned int i = 0; i < object->getChildCount(); i++ ) 

		createCityObject( object->getChild(i), settings, grp );
#endif

	return true;
}

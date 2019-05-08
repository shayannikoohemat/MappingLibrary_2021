/*---------------------------------------------------------
|Initial creation: Biao Xiong
|Data: July 28, 2011
|
|
|
-----------------------------------------------------------*/
// This is the implementation file for TinyGML parser



#include "parser.h"
#include "tinyxml.h"
//#define TIXML_USE_STL;

using namespace citygml;

// CityGML LibXml2 SAX parsing handler
#define MAX_ATTRIBUTE_NUM 40
#define MAX_ATTRIBUTE_LENGTH 256
	


class CityGMLHandlerTinyXml : public CityGMLHandler
{
public:
	CityGMLHandlerTinyXml( const ParserParams& params ) : CityGMLHandler( params ) {}

	void startElement( const char* name, char** attrs ) {
		CityGMLHandler::startElement( wstos( name ), attrs );
	}
	void endElement( const char* name ) {
		CityGMLHandler::endElement( wstos( name ) );
	}

	void characters( const char *chars, int length ) {
		for ( int i = 0; i < length; i++ ) _buff << (char)chars[i]; 
	}
	static inline std::string wstos( const char* const str ) {
		return std::string( (const char*)str );
	}

protected:
	std::string getAttribute( void* attributes, const std::string& attname, const std::string& defvalue = "" ) 	{
		const char **attrs = (const char**)attributes;
		if ( !attrs ) return "";
		for ( int i = 0; attrs[i] != NULL; i += 2 ) 
			if ( wstos( attrs[i] ) == attname ) return wstos( attrs[ i + 1 ] );
		return defvalue;
	}
};


class CityGmlVisitor : public TiXmlVisitor
{
public:
	CityGmlVisitor(CityGMLHandlerTinyXml* handler);
	~CityGmlVisitor(){};

	/// Visit a document.
	inline bool VisitEnter( const TiXmlDocument& /*doc*/ );
	/// Visit a document.
	inline bool VisitExit( const TiXmlDocument& /*doc*/ );

	/// Visit an element.
	bool VisitEnter( const TiXmlElement& /*element*/, const TiXmlAttribute* /*firstAttribute*/ );
	/// Visit an element.
	bool VisitExit( const TiXmlElement& /*element*/ );

	/// Visit a declaration
	bool Visit( const TiXmlDeclaration& /*declaration*/ )	{ return true; }
	/// Visit a text node
	bool Visit( const TiXmlText& /*text*/ )					{ return true; }
	/// Visit a comment node
	bool Visit( const TiXmlComment& /*comment*/ )			{ return true; }
	/// Visit an unknown node
	bool Visit( const TiXmlUnknown& /*unknown*/ )			{ return true; }
private:
	CityGmlVisitor() {};

public:
	inline bool SetGMLParameter(const ParserParams& params) {_params = params;}
private:
	ParserParams _params;
	CityGMLHandlerTinyXml* _handler;
};

CityGmlVisitor::CityGmlVisitor(CityGMLHandlerTinyXml* handler)
{
	if (handler)
		_handler = handler;
	else 
		_handler = 0;
}

bool CityGmlVisitor::VisitEnter(const TiXmlDocument& doc)
{
	return true;
}

bool CityGmlVisitor::VisitExit( const TiXmlDocument& doc )
{
	return true;
}

/// Visit an element.
bool CityGmlVisitor::VisitEnter( const TiXmlElement& element, const TiXmlAttribute* firstAttribute )
{
	if (!_handler) return false;
	
	const char* attriName;
	const char* attriValue;
	const TiXmlAttribute* pXmlAttri = firstAttribute;
	int iAttri = 0;
	char** attris = 0;

	if (pXmlAttri)	{
		attris = new char* [MAX_ATTRIBUTE_NUM];
		for (int i=0; i<MAX_ATTRIBUTE_NUM; ++i) {
			attris[i] = new char [MAX_ATTRIBUTE_LENGTH];
		}
	}
	
	while (pXmlAttri) {
		//do something
		if (2*iAttri>MAX_ATTRIBUTE_NUM) {
			printf( "Too much attribute in %s\n", element.Value() );
			break;//too much attribute
		}
		attriName = pXmlAttri->Name();
		attriValue = pXmlAttri->Value();
		memcpy(attris[2*iAttri], attriName, strlen(attriName)*sizeof(char)+1);
		memcpy(attris[2*iAttri+1], attriValue, strlen(attriValue)*sizeof(char)+1);
	//	strcast();

		pXmlAttri = pXmlAttri->Next();
		iAttri++;
	}


	if (0 == strcmp( element.Value(), "gml:posList")) {
		int aaa = 0;
	}

	const char* xmlText = element.GetText();
	if (xmlText) 
		_handler->characters(xmlText, strlen(xmlText));
	_handler->startElement(element.Value(), attris);

	//free memory
	if (attris)	{
		for (int i=0; i<MAX_ATTRIBUTE_NUM; ++i) {
			if (attris[i])
				delete [] attris[i];
		}
		delete [] attris;
	}
	
	return true;
}

/// Visit an element.
bool CityGmlVisitor::VisitExit( const TiXmlElement& element )
{
	if (!_handler) return false;
	_handler->endElement(element.Value());
	return true;
}

// Parsing methods
namespace citygml {

	CityModel* load( const char* fileName, const ParserParams& params )
	{
		TiXmlDocument xmlDoc(fileName);
		if(!xmlDoc.LoadFile())	{
			printf( "Error in %s: %s\n", xmlDoc.Value(), xmlDoc.ErrorDesc() );
			exit( 1 );
		}

		CityGMLHandlerTinyXml* handle = new CityGMLHandlerTinyXml(params);
		CityGmlVisitor visitor(handle);
		xmlDoc.Accept(static_cast<TiXmlVisitor*>(&visitor));
		CityModel* cityModel = handle->getModel();
		int n = cityModel->size();
		delete handle;
		return cityModel;
		//xmlDoc.Parse();
	}
}


//*********************************************************
//GLFONT.CPP -- glFont routines
//Copyright (c) 1998 Brad Fish
//Copyright (c) 2002 Henri Kyrki
//See glFont.txt for terms of use
//10.5 2002
//*********************************************************

#include <stdio.h>
#include <stdlib.h>
#include "glfont.h"
#include "Image.h"

//*********************************************************
//GLFontBase
//*********************************************************

GLFontBase::GLFontBase() : ok(FALSE)
{
}

void GLFontBase::CreateImpl(const std::string &Filename, GLuint Tex, bool PixelPerfect)
{
	Font.Char = NULL;
	FreeResources();

	FILE *Input;

	//Open font file
	if ((Input = fopen(Filename.c_str(), "rb")) == NULL)
		throw GLFontError::InvalidFile();

	//Read glFont structure
	fread(&Font, sizeof(GLFONT), 1, Input);

	//Save texture number
	Font.Tex = Tex;

	//Get number of characters
	int Num = Font.IntEnd - Font.IntStart + 1;

	//Allocate memory for characters
	//if ((Font.Char = (GLFONTCHAR *)malloc(sizeof(GLFONTCHAR) * Num)) == NULL)
	Font.Char = new GLFONTCHAR[Num];

	//Read glFont characters
	fread(Font.Char, sizeof(GLFONTCHAR), Num, Input);

	//Get texture size
	Num = Font.TexWidth * Font.TexHeight * 2;
    
	//Allocate memory for texture data
	//TexBytes = (char *)malloc(Num)
	char *TexBytes = new char[Num];

	//Read texture data
	fread(TexBytes, sizeof(char), Num, Input);

	//Set texture attributes
	glBindTexture(GL_TEXTURE_2D, Font.Tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	if(PixelPerfect)
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	}
	else
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	}
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	  
	//Create texture
	glTexImage2D(GL_TEXTURE_2D, 0, 2, Font.TexWidth, Font.TexHeight, 0, 
	             GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, (void *)TexBytes);

	//Clean up
	delete []TexBytes;
	fclose(Input);

	ok = TRUE;
}
//*********************************************************
void GLFontBase::FreeResources ()
{
	//Free character memory
	if (Font.Char != NULL) delete []Font.Char;
	ok = FALSE;
}
//*********************************************************
void GLFontBase::Begin ()
{
	if (!ok)
	{
		throw GLFontError::InvalidFont();
	}

	glBindTexture(GL_TEXTURE_2D, Font.Tex);
}
//*********************************************************
GLFontBase::~GLFontBase ()
{
	FreeResources();
}

//*********************************************************
//PixelPerfectGLFont
//*********************************************************

PixelPerfectGLFont::PixelPerfectGLFont()
{
}
//*********************************************************
void PixelPerfectGLFont::Create(const std::string &Filename, GLuint Tex)
{
	GLFontBase::CreateImpl(Filename, Tex, TRUE);
	for (int i = 0; i < Font.IntEnd - Font.IntStart + 1; i++)
	{
		Font.Char[i].width = (int)((Font.Char[i].tx2 - Font.Char[i].tx1)*Font.TexWidth);
		Font.Char[i].height = (int)((Font.Char[i].ty2 - Font.Char[i].ty1)*Font.TexHeight);
	}
}
//*********************************************************
void PixelPerfectGLFont::TextOut (std::string String, int x, int y, int z)
{
	//Return if we don't have a valid glFont
	if (!ok)
	{
		throw GLFontError::InvalidFont();
	}

	//Get length of string
	int Length = String.length();

	//Begin rendering quads
	glBegin(GL_QUADS);

	//Loop through characters
	for (int i = 0; i < Length; i++)
	{
		//Get pointer to glFont character
		GLFONTCHAR *Char = &Font.Char[(int)String[i] - Font.IntStart];

		//Specify vertices and texture coordinates
		glTexCoord2f(Char->tx1, Char->ty1);
		glVertex3i(x, y + Char->height, z);
		glTexCoord2f(Char->tx1, Char->ty2);
		glVertex3i(x, y, z);
		glTexCoord2f(Char->tx2, Char->ty2);
		glVertex3i(x + Char->width, y, z);
		glTexCoord2f(Char->tx2, Char->ty1);
		glVertex3i(x + Char->width, y + Char->height, z);

		//Move to next character
		x += Char->width;
	}

	//Stop rendering quads
	glEnd();
}

//*********************************************************
//GLFont
//*********************************************************

GLFont::GLFont()
{
}
//*********************************************************
void GLFont::Create(const std::string &Filename, GLuint Tex)
{
	GLFontBase::CreateImpl(Filename, Tex, FALSE);
}
//*********************************************************
void GLFont::TextOutF(std::string String, float x, float y, float z, float scale)
{
	//Return if we don't have a valid glFont
	if (!ok)
	{
		throw GLFontError::InvalidFont();
	}

	//Get length of string
	int Length = String.length();

	//Begin rendering quads
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBegin(GL_QUADS);

	//Loop through characters
	for (int i = 0; i < Length; i++)
	{
		//Get pointer to glFont character
		GLFONTCHAR *Char = &Font.Char[(int)String[i] - Font.IntStart];
		//Specify vertices and texture coordinates
		glTexCoord2f(Char->tx1, Char->ty1);
		glVertex3f(x, y + Char->dy / scale, z);
		glTexCoord2f(Char->tx1, Char->ty2);
		glVertex3f(x, y, z);
		glTexCoord2f(Char->tx2, Char->ty2);
		glVertex3f(x + Char->dx / scale, y, z);
		glTexCoord2f(Char->tx2, Char->ty1);
		glVertex3f(x + Char->dx / scale, y + Char->dy / scale, z);

		//Move to next character
		x += Char->dx / scale;
	}

	//Stop rendering quads
	glEnd();
}

//End of file




/*
 * This program source code file is part of kicad2mcad
 *
 * Copyright (C) 2016 Cirilo Bernardo <cirilo.bernardo@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you may find one here:
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 * or you may search the http://www.gnu.org website for the version 2 license,
 * or you may write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#define  GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <sstream>
#include <string>
#include <wx/filename.h>
#include <wx/log.h>

#include "oce_utils.h"
#include "kicadpad.h"

#include <IGESCAFControl_Reader.hxx>
#include <IGESCAFControl_Writer.hxx>
#include <Interface_Static.hxx>
#include <Quantity_Color.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDataStd_Name.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDF_ChildIterator.hxx>
#include <TopExp_Explorer.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ColorTool.hxx>

#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepAlgoAPI_Cut.hxx>

#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Builder.hxx>

#define USER_PREC (1e-4)
#define USER_ANGLE_PREC (1e-6)
// minimum PCB thickness in mm (2 microns assumes a very thin polyimide film)
#define THICKNESS_MIN (0.002)
// default PCB thickness in mm
#define THICKNESS_DEFAULT (1.6)
// nominal offset from the board
#define BOARD_OFFSET (0.05 )

// supported file types
enum FormatType
{
    FMT_NONE = 0,
    FMT_STEP = 1,
    FMT_IGES = 2,
    FMT_EMN  = 3,
    FMT_IDF  = 4
};


FormatType fileType( const char* aFileName )
{
    wxFileName lfile( aFileName );

    if( !lfile.FileExists() )
    {
        std::ostringstream ostr;
        ostr << __FILE__ << ": " << __FUNCTION__ << ": " << __LINE__ << "\n";
        ostr << "  * no such file: '" << aFileName << "'\n";
        wxLogMessage( "%s\n", ostr.str().c_str() );

        return FMT_NONE;
    }

    wxString ext = lfile.GetExt();

    if( ext == "idf" || ext == "IDF" )
        return FMT_IDF;     // component outline
    else if( ext == "emn" || ext == "EMN" )
        return FMT_EMN;     // PCB assembly

    std::ifstream ifile;
    ifile.open( aFileName );

    if( !ifile.is_open() )
        return FMT_NONE;

    char iline[82];
    memset( iline, 0, 82 );
    ifile.getline( iline, 82 );
    ifile.close();
    iline[81] = 0;  // ensure NULL termination when string is too long

    // check for STEP in Part 21 format
    // (this can give false positives since Part 21 is not exclusively STEP)
    if( !strncmp( iline, "ISO-10303-21;", 13 ) )
        return FMT_STEP;

    std::string fstr = iline;

    // check for STEP in XML format
    // (this can give both false positive and false negatives)
    if( fstr.find( "urn:oid:1.0.10303." ) != std::string::npos )
        return FMT_STEP;

    // Note: this is a very simple test which can yield false positives; the only
    // sure method for determining if a file *not* an IGES model is to attempt
    // to load it.
    if( iline[72] == 'S' && ( iline[80] == 0 || iline[80] == 13 || iline[80] == 10 ) )
        return FMT_IGES;

    return FMT_NONE;
}


PCBMODEL::PCBMODEL()
{
    m_app = XCAFApp_Application::GetApplication();
    m_app->NewDocument( "MDTV-XCAF", m_doc );
    m_assy = XCAFDoc_DocumentTool::ShapeTool ( m_doc->Main() );
    m_assy_label = m_assy->NewShape();
    m_hasPCB = false;
    m_components = 0;
    m_precision = USER_PREC;
    m_angleprec = USER_ANGLE_PREC;
    m_thickness = THICKNESS_DEFAULT;
    return;
}


PCBMODEL::~PCBMODEL()
{
    m_doc->Close();
    return;
}

// add an outline segment
bool PCBMODEL::AddOutlineSegment( KICADCURVE* aCurve )
{
    // XXX - TO BE IMPLEMENTED
    return false;
}


// add a pad hole or slot
bool PCBMODEL::AddPadHole( KICADPAD* aPad )
{
    // XXX - TO BE IMPLEMENTED
    return false;
}


// add a component at the given position and orientation
bool PCBMODEL::AddComponent( const std::string& aFileName, const std::string aRefDes,
    bool aBottom, DOUBLET aPosition, double aRotation,
    TRIPLET aOffset, TRIPLET aOrientation )
{
    // first retrieve a label
    TDF_Label lmodel;

    if( !getModelLabel( aFileName, lmodel ) )
    {
        std::cerr << "** XXX: NO LABEL\n";
        return false;
    }

    // calculate the Location transform
    TopLoc_Location toploc;

    if( !getModelLocation( aBottom, aPosition, aRotation, aOffset, aOrientation, toploc ) )
    {
        std::cerr << "** XXX: NO LOCATION\n";
        return false;
    }

    // add the located sub-assembly
    TDF_Label llabel = m_assy->AddComponent( m_assy_label, lmodel, toploc );

    if( llabel.IsNull() )
    {
        std::cerr << "** XXX: FAIL (could not add component)\n";
        return false;
    }

    // attach the RefDes name
    TCollection_ExtendedString refdes( aRefDes.c_str() );
    TDataStd_Name::Set( llabel, refdes );

    return true;
}


void PCBMODEL::SetPCBThickness( double aThickness )
{
    if( aThickness < 0.0 )
        m_thickness = THICKNESS_DEFAULT;
    else if( aThickness < THICKNESS_MIN )
        m_thickness = THICKNESS_MIN;
    else
        m_thickness = aThickness;

    return;
}


// create the PCB (board only) model using the current outlines and drill holes
bool PCBMODEL::CreatePCB()
{
    if( m_hasPCB )
    {
        if( m_pcb_label.IsNull() )
            return false;

        return true;
    }

    // XXX - TO BE IMPLEMENTED
    return true;
}


// write the assembly model in IGES format
bool PCBMODEL::WriteIGES( const std::string& aFileName, bool aOverwrite )
{
    // XXX - check that we have a valid assembly

    IGESCAFControl_Writer writer;
    writer.SetColorMode( Standard_True );
    writer.SetNameMode( Standard_True );

    if( Standard_False == writer.Transfer( m_doc )
        || Standard_False == writer.Write( aFileName.c_str() ) )
        return false;

    return true;
}


// write the assembly model in STEP format
bool PCBMODEL::WriteSTEP( const std::string& aFileName, bool aOverwrite )
{
    // XXX - check that we have a valid assembly

    STEPCAFControl_Writer writer;
    writer.SetColorMode( Standard_True );
    writer.SetNameMode( Standard_True );

    if( Standard_False == writer.Transfer( m_doc, STEPControl_AsIs )
        || Standard_False == writer.Write( aFileName.c_str() ) )
        return false;

    return true;
}


bool PCBMODEL::getModelLabel( const std::string aFileName, TDF_Label& aLabel )
{
    MODEL_MAP::const_iterator mm = m_models.find( aFileName );

    if( mm != m_models.end() )
    {
        aLabel = mm->second;
        return true;
    }

    aLabel.Nullify();

    Handle( TDocStd_Document )  doc;
    m_app->NewDocument( "MDTV-XCAF", doc );

    FormatType modelFmt = fileType( aFileName.c_str() );

    switch( modelFmt )
    {
        case FMT_IGES:
            if( !readIGES( doc, aFileName.c_str() ) )
            {
                std::cerr << "** readIGES() failed\n";
                return false;
            }
            break;

        case FMT_STEP:
            if( !readSTEP( doc, aFileName.c_str() ) )
            {
                std::cerr << "** readSTEP() failed\n";
                return false;
            }
            break;

        // TODO: implement IDF and EMN converters

        default:
            return false;
    }

    aLabel = transferModel( doc, m_doc );

    if( aLabel.IsNull() )
    {
        std::cerr << "** XXX: FAILED\n";
        return false;
    }

    // attach the PART NAME ( base filename: note that in principle
    // different models may have the same base filename )
    wxFileName afile( aFileName.c_str() );
    std::string pname( afile.GetName().ToUTF8() );
    TCollection_ExtendedString partname( pname.c_str() );
    TDataStd_Name::Set( aLabel, partname );

    m_models.insert( MODEL_DATUM( aFileName, aLabel ) );
    ++m_components;
    return true;
}


bool PCBMODEL::getModelLocation( bool aBottom, DOUBLET aPosition, double aRotation,
    TRIPLET aOffset, TRIPLET aOrientation, TopLoc_Location& aLocation )
{
    // Order of operations:
    // a. aOrientation is applied -Z*-Y*-X
    // b. aOffset is applied
    //      Top ? add thickness to the Z offset
    // c. Bottom ? Rotate on X axis (in contrast to most ECAD which mirror on Y),
    //             then rotate on +Z
    //    Top ? rotate on -Z
    // d. aPosition is applied
    //
    // Note: Y axis is inverted in KiCad

    gp_Trsf lPos;
    lPos.SetTranslation( gp_Vec( aPosition.x, -aPosition.y, 0.0 ) );

    // offset (inches)
    aOffset.x *= 25.4;
    aOffset.y *= -25.4;
    aOffset.z *= 25.4 + BOARD_OFFSET;
    gp_Trsf lRot;

    if( aBottom )
    {
        lRot.SetRotation( gp_Ax1( gp_Pnt( 0.0, 0.0, 0.0 ), gp_Dir( 0.0, 0.0, 1.0 ) ), aRotation );
        lPos.Multiply( lRot );
        lRot.SetRotation( gp_Ax1( gp_Pnt( 0.0, 0.0, 0.0 ), gp_Dir( 1.0, 0.0, 0.0 ) ), M_PI );
        lPos.Multiply( lRot );
    }
    else
    {
        aOffset.z += m_thickness;
        lRot.SetRotation( gp_Ax1( gp_Pnt( 0.0, 0.0, 0.0 ), gp_Dir( 0.0, 0.0, 1.0 ) ), aRotation );
        lPos.Multiply( lRot );
    }

    gp_Trsf lOff;
    lOff.SetTranslation( gp_Vec( aOffset.x, aOffset.y, aOffset.z ) );
    lPos.Multiply( lOff );

    gp_Trsf lOrient;
    lOrient.SetRotation( gp_Ax1( gp_Pnt( 0.0, 0.0, 0.0 ),
       gp_Dir( 0.0, 0.0, 1.0 ) ), -aOrientation.z );
    lPos.Multiply( lOrient );
    lOrient.SetRotation( gp_Ax1( gp_Pnt( 0.0, 0.0, 0.0 ),
        gp_Dir( 0.0, 1.0, 0.0 ) ), -aOrientation.y );
    lPos.Multiply( lOrient );
    lOrient.SetRotation( gp_Ax1( gp_Pnt( 0.0, 0.0, 0.0 ),
        gp_Dir( 1.0, 0.0, 0.0 ) ), -aOrientation.x );
    lPos.Multiply( lOrient );

    aLocation = TopLoc_Location( lPos );
    return true;
}


bool PCBMODEL::readIGES( Handle( TDocStd_Document )& doc, const char* fname )
{
    IGESCAFControl_Reader reader;
    IFSelect_ReturnStatus stat  = reader.ReadFile( fname );

    if( stat != IFSelect_RetDone )
        return false;

    // Enable user-defined shape precision
    if( !Interface_Static::SetIVal( "read.precision.mode", 1 ) )
        return false;

    // Set the shape conversion precision to USER_PREC (default 0.0001 has too many triangles)
    if( !Interface_Static::SetRVal( "read.precision.val", USER_PREC ) )
        return false;

    // set other translation options
    reader.SetColorMode(true);  // use model colors
    reader.SetNameMode(false);  // don't use IGES label names
    reader.SetLayerMode(false); // ignore LAYER data

    if ( !reader.Transfer( doc ) )
    {
        doc->Close();
        return false;
    }

    // are there any shapes to translate?
    if( reader.NbShapes() < 1 )
    {
        doc->Close();
        return false;
    }

    return true;
}


bool PCBMODEL::readSTEP( Handle(TDocStd_Document)& doc, const char* fname )
{
    STEPCAFControl_Reader reader;
    IFSelect_ReturnStatus stat  = reader.ReadFile( fname );

    if( stat != IFSelect_RetDone )
        return false;

    // Enable user-defined shape precision
    if( !Interface_Static::SetIVal( "read.precision.mode", 1 ) )
        return false;

    // Set the shape conversion precision to USER_PREC (default 0.0001 has too many triangles)
    if( !Interface_Static::SetRVal( "read.precision.val", USER_PREC ) )
        return false;

    // set other translation options
    reader.SetColorMode(true);  // use model colors
    reader.SetNameMode(false);  // don't use label names
    reader.SetLayerMode(false); // ignore LAYER data

    if ( !reader.Transfer( doc ) )
    {
        doc->Close();
        return false;
    }

    // are there any shapes to translate?
    if( reader.NbRootsForTransfer() < 1 )
    {
        doc->Close();
        return false;
    }

    return true;
}


TDF_Label PCBMODEL::transferModel( Handle( TDocStd_Document )& source,
    Handle( TDocStd_Document )& dest )
{
    // transfer data from Source into a top level component of Dest

    // s_assy = shape tool for the source
    Handle(XCAFDoc_ShapeTool) s_assy = XCAFDoc_DocumentTool::ShapeTool ( source->Main() );

    // retrieve all free shapes within the assembly
    TDF_LabelSequence frshapes;
    s_assy->GetFreeShapes( frshapes );

    // d_assy = shape tool for the destination
    Handle(XCAFDoc_ShapeTool) d_assy = XCAFDoc_DocumentTool::ShapeTool ( dest->Main() );

    // create a new shape within the destination and set the assembly tool to point to it
    TDF_Label component = d_assy->NewShape();

    int nshapes = frshapes.Length();
    int id = 1;
    Handle( XCAFDoc_ColorTool ) scolor = XCAFDoc_DocumentTool::ColorTool( source->Main() );
    Handle( XCAFDoc_ColorTool ) dcolor = XCAFDoc_DocumentTool::ColorTool( dest->Main() );
    TopExp_Explorer dtop;
    TopExp_Explorer stop;

    while( id <= nshapes )
    {
        TopoDS_Shape shape = s_assy->GetShape( frshapes.Value(id) );

        if ( !shape.IsNull() )
        {
            TDF_Label niulab = d_assy->AddComponent( component, shape, Standard_False );

            // check for per-surface colors
            stop.Init( shape, TopAbs_FACE );
            dtop.Init( d_assy->GetShape( niulab ), TopAbs_FACE );

            while( stop.More() && dtop.More() )
            {
                Quantity_Color face_color;

                TDF_Label tl;

                // give priority to the base shape's color
                if( s_assy->FindShape( stop.Current(), tl ) )
                {
                    if( scolor->GetColor( tl, XCAFDoc_ColorSurf, face_color )
                        || scolor->GetColor( tl, XCAFDoc_ColorGen, face_color )
                        || scolor->GetColor( tl, XCAFDoc_ColorCurv, face_color ) )
                    {
                        dcolor->SetColor( dtop.Current(), face_color, XCAFDoc_ColorSurf );
                    }
                }
                else  if( scolor->GetColor( stop.Current(), XCAFDoc_ColorSurf, face_color )
                          || scolor->GetColor( stop.Current(), XCAFDoc_ColorGen, face_color )
                          || scolor->GetColor( stop.Current(), XCAFDoc_ColorCurv, face_color ) )
                {

                    dcolor->SetColor( dtop.Current(), face_color, XCAFDoc_ColorSurf );
                }

                stop.Next();
                dtop.Next();
            }

            // check for per-solid colors
            stop.Init( shape, TopAbs_SOLID );
            dtop.Init( d_assy->GetShape( niulab ), TopAbs_SOLID );

            while( stop.More() && dtop.More() )
            {
                Quantity_Color face_color;

                TDF_Label tl;

                // give priority to the base shape's color
                if( s_assy->FindShape( stop.Current(), tl ) )
                {
                    if( scolor->GetColor( tl, XCAFDoc_ColorSurf, face_color )
                        || scolor->GetColor( tl, XCAFDoc_ColorGen, face_color )
                        || scolor->GetColor( tl, XCAFDoc_ColorCurv, face_color ) )
                    {
                        dcolor->SetColor( dtop.Current(), face_color, XCAFDoc_ColorGen );
                    }
                }
                else  if( scolor->GetColor( stop.Current(), XCAFDoc_ColorSurf, face_color )
                          || scolor->GetColor( stop.Current(), XCAFDoc_ColorGen, face_color )
                          || scolor->GetColor( stop.Current(), XCAFDoc_ColorCurv, face_color ) )
                {

                    dcolor->SetColor( dtop.Current(), face_color, XCAFDoc_ColorGen );
                }

                stop.Next();
                dtop.Next();
            }


        }

        ++id;
    };

    return component;
}

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
#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <utility>
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
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepAlgoAPI_Cut.hxx>

#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Builder.hxx>

#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#define USER_PREC (1e-4)
#define USER_ANGLE_PREC (1e-6)
// minimum PCB thickness in mm (2 microns assumes a very thin polyimide film)
#define THICKNESS_MIN (0.002)
// default PCB thickness in mm
#define THICKNESS_DEFAULT (1.6)
// nominal offset from the board
#define BOARD_OFFSET (0.05 )
// min. length**2 below which 2 points are considered coincident
#define MIN_LENGTH2 (0.0001)

// XXX - TEST ONLY
static void printSegs( std::list< KICADCURVE >& curves );

static void getEndPoints( const KICADCURVE& aCurve, double& spx0, double& spy0,
    double& epx0, double& epy0 )
{
    if( CURVE_ARC == aCurve.m_form )
    {
        spx0 = aCurve.m_end.x;
        spy0 = aCurve.m_end.y;
        epx0 = aCurve.m_ep.x;
        epy0 = aCurve.m_ep.y;
        return;
    }

    // assume a line
    spx0 = aCurve.m_start.x;
    spy0 = aCurve.m_start.y;
    epx0 = aCurve.m_end.x;
    epy0 = aCurve.m_end.y;
    return;
}

static void getCurveEndPoint( const KICADCURVE& aCurve, DOUBLET& aEndPoint )
{
    if( CURVE_CIRCLE == aCurve.m_form )
        return; // circles are closed loops and have no end point

    if( CURVE_ARC == aCurve.m_form )
    {
        aEndPoint.x = aCurve.m_ep.x;
        aEndPoint.y = aCurve.m_ep.y;
        return;
    }

    // assume a line
    aEndPoint.x = aCurve.m_end.x;
    aEndPoint.y = aCurve.m_end.y;
    return;
}

static void reverseCurve( KICADCURVE& aCurve )
{
    if( CURVE_NONE ==  aCurve.m_form || CURVE_CIRCLE == aCurve.m_form )
        return;

    if( CURVE_LINE == aCurve.m_form )
    {
        std::swap( aCurve.m_start, aCurve.m_end );
        return;
    }

    std::swap( aCurve.m_end, aCurve.m_ep );
    aCurve.m_angle = -aCurve.m_angle;

    return;
}


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
    m_minx = 1.0e10;    // absurdly large number; any valid PCB X value will be smaller
    m_mincurve = m_curves.end();
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
    if( NULL == aCurve || LAYER_EDGE != aCurve->m_layer || CURVE_NONE == aCurve->m_form )
        return false;

    if( CURVE_LINE != aCurve->m_form )
    {
        // ensure that the start and end are not the same point
        double dx = aCurve->m_end.x - aCurve->m_start.x;
        double dy = aCurve->m_end.y - aCurve->m_start.y;
        double rad = dx * dx + dy * dy;

        if( rad < MIN_LENGTH2 )
            return false;

        // calculate the radius and, if applicable, end point
        rad = sqrt( rad );
        aCurve->m_radius = rad;

        if( CURVE_ARC == aCurve->m_form )
        {
            double eang = atan2( dy, dx ) + aCurve->m_angle;

            aCurve->m_ep.x = aCurve->m_start.x + rad * cos( eang );
            aCurve->m_ep.y = aCurve->m_start.y + rad * sin( eang );
        }
    }

    m_curves.push_back( *aCurve );

    // check if this curve has the current leftmost feature
    switch( aCurve->m_form )
    {
        case CURVE_LINE:
            if( aCurve->m_start.x < m_minx )
            {
                m_minx = aCurve->m_start.x;
                m_mincurve = --(m_curves.end());
            }

            if( aCurve->m_end.x < m_minx )
            {
                m_minx = aCurve->m_end.x;
                m_mincurve = --(m_curves.end());
            }

            break;

        case CURVE_CIRCLE:
            do
            {
                double dx = aCurve->m_start.x - aCurve->m_radius;

                if( dx < m_minx )
                {
                    m_minx = dx;
                    m_mincurve = --(m_curves.end());
                }
            } while( 0 );

            break;

        case CURVE_ARC:
            do
            {
                double dx0 = aCurve->m_end.x - aCurve->m_start.x;
                double dy0 = aCurve->m_end.y - aCurve->m_start.y;
                int q0;  // quadrant of start point

                if( dx0 > 0.0 && dy0 >= 0.0 )
                    q0 = 1;
                else if( dx0 <= 0.0 && dy0 > 0.0 )
                    q0 = 2;
                else if( dx0 < 0.0 && dy0 <= 0.0 )
                    q0 = 3;
                else
                    q0 = 4;

                double dx1 = aCurve->m_ep.x - aCurve->m_start.x;
                double dy1 = aCurve->m_ep.y - aCurve->m_start.y;
                int q1;  // quadrant of end point

                if( dx1 > 0.0 && dy1 >= 0.0 )
                    q1 = 1;
                else if( dx1 <= 0.0 && dy1 > 0.0 )
                    q1 = 2;
                else if( dx1 < 0.0 && dy1 <= 0.0 )
                    q1 = 3;
                else
                    q1 = 4;

                // calculate x0, y0 for the start point on a CCW arc
                double x0 = aCurve->m_end.x;
                double x1 = aCurve->m_ep.x;

                if( aCurve->m_angle < 0.0 )
                {
                    std::swap( q0, q1 );
                    std::swap( x0, x1 );
                }

                double minx;

                if( ( q0 <= 2 && q1 >= 3 ) || ( q0 >= 3 && x0 > x1 ) )
                    minx = aCurve->m_start.x - aCurve->m_radius;
                else
                    minx = std::min( x0, x1 );

                if( minx < m_minx )
                {
                    m_minx = minx;
                    m_mincurve = --(m_curves.end());
                }

            } while( 0 );

            break;

        default:
            // unexpected curve type
            do
            {
                std::ostringstream ostr;
                ostr << __FILE__ << ": " << __FUNCTION__ << ": " << __LINE__ << "\n";
                ostr << "  * unsupported curve type: '" << aCurve->m_form << "'\n";
                wxLogMessage( "%s\n", ostr.str().c_str() );
            } while( 0 );

            return false;
    }

    std::cerr << "** XXX: Added a segment [";
    switch( aCurve->m_form )
    {
        case CURVE_LINE:
            std::cerr << "LINE]\n";
            break;

        case CURVE_CIRCLE:
            std::cerr << "CIRCLE]\n";
            break;

        case CURVE_ARC:
            std::cerr << "ARC]\n";
            break;

        default:
            std::cerr << "UNKNOWN]\n";
            break;
    }

    return true;
}


// add a pad hole or slot
bool PCBMODEL::AddPadHole( KICADPAD* aPad )
{
    if( NULL == aPad || !aPad->IsThruHole() )
        return false;

    if( !aPad->m_drill.oval )
    {
        TopoDS_Shape s = BRepPrimAPI_MakeCylinder( aPad->m_drill.size.x, m_thickness * 2.0 ).Shape();
        gp_Trsf shift;
        shift.SetTranslation( gp_Vec( aPad->m_position.x, aPad->m_position.y, -m_thickness * 0.5 ) );
        BRepBuilderAPI_Transform hole( s, shift );
        m_cutouts.push_back( hole.Shape() );
        return true;
    }

    // XXX - TO BE IMPLEMENTED: slotted hole
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

    if( m_mincurve == m_curves.end() )
    {
        m_hasPCB = true;
        std::ostringstream ostr;
        ostr << __FILE__ << ": " << __FUNCTION__ << ": " << __LINE__ << "\n";
        ostr << "  * no valid board outline\n";
        wxLogMessage( "%s\n", ostr.str().c_str() );
        return false;
    }

    // XXX - START CHECK BLOCK
    std::cerr << "** XXX: MIN Curve: ";

    switch( m_mincurve->m_form )
    {
        case CURVE_LINE:
            std::cerr << "LINE\n";
            break;

        case CURVE_CIRCLE:
            std::cerr << "CIRCLE\n";
            break;

        case CURVE_ARC:
            std::cerr << "ARC\n";
            break;

        default:
            std::cerr << "UNKNOWN\n";
            break;
    }

    std::cerr << "   start(" << m_mincurve->m_start.x << ", " << m_mincurve->m_start.y << ")\n";
    std::cerr << "     end(" << m_mincurve->m_end.x << ", " << m_mincurve->m_end.y << ")\n";

    if( CURVE_LINE != m_mincurve->m_form )
    {
        std::cerr << "     rad: " << m_mincurve->m_radius << "\n";

        if( CURVE_ARC == m_mincurve->m_form )
            std::cerr << "   angle: " << (m_mincurve->m_angle * 180.0 / M_PI) << "\n";

    }
    // XXX - END CHECK BLOCK

    m_hasPCB = true;    // whether or not operations fail we note that CreatePCB has been invoked
    TopoDS_Shape board;
    OUTLINE oln;    // loop to assemble (represents PCB outline and cutouts)
    oln.AddSegment( *m_mincurve );
    m_curves.erase( m_mincurve );

    while( !m_curves.empty() )
    {
        if( oln.IsClosed() )
        {
            if( board.IsNull() )
            {
                if( !oln.MakeShape( board, m_thickness ) )
                {
                    std::ostringstream ostr;
                    ostr << __FILE__ << ": " << __FUNCTION__ << ": " << __LINE__ << "\n";
                    ostr << "  * could not create board extrusion\n";
                    wxLogMessage( "%s\n", ostr.str().c_str() );

                    return false;
                }
            }
            else
            {
                TopoDS_Shape hole;

                if( oln.MakeShape( board, m_thickness ) )
                {
                    m_cutouts.push_back( hole );
                }
                else
                {
                    std::ostringstream ostr;
                    ostr << __FILE__ << ": " << __FUNCTION__ << ": " << __LINE__ << "\n";
                    ostr << "  * could not create board cutout\n";
                    wxLogMessage( "%s\n", ostr.str().c_str() );
                }
            }

            oln.Clear();
            oln.AddSegment( m_curves.front() );
            m_curves.pop_front();
            continue;
        }

        std::list< KICADCURVE >::iterator sC = m_curves.begin();
        std::list< KICADCURVE >::iterator eC = m_curves.end();

        while( sC != eC )
        {
            if( oln.AddSegment( *sC ) )
            {
                m_curves.erase( sC );
                break;
            }

            ++sC;
        }

        if( sC == eC )
        {
            std::ostringstream ostr;
            ostr << __FILE__ << ": " << __FUNCTION__ << ": " << __LINE__ << "\n";
            ostr << "  * could not close outline (dropping outline data with " << oln.m_curves.size() << " segments)\n";
            printSegs( oln.m_curves );
            wxLogMessage( "%s\n", ostr.str().c_str() );
            oln.Clear();
            oln.AddSegment( m_curves.front() );
            m_curves.pop_front();
        }
    }

    if( oln.IsClosed() )
    {
        if( board.IsNull() )
        {
            if( !oln.MakeShape( board, m_thickness ) )
            {
                std::ostringstream ostr;
                ostr << __FILE__ << ": " << __FUNCTION__ << ": " << __LINE__ << "\n";
                ostr << "  * could not create board extrusion\n";
                wxLogMessage( "%s\n", ostr.str().c_str() );
                return false;
            }
        }
        else
        {
            TopoDS_Shape hole;

            if( oln.MakeShape( board, m_thickness ) )
            {
                m_cutouts.push_back( hole );
            }
            else
            {
                std::ostringstream ostr;
                ostr << __FILE__ << ": " << __FUNCTION__ << ": " << __LINE__ << "\n";
                ostr << "  * could not create board cutout\n";
                wxLogMessage( "%s\n", ostr.str().c_str() );
            }
        }
    }

    // subtract cutouts (if any)
    if( !m_cutouts.empty() )
    {
        BRep_Builder    bld;
        TopoDS_Compound holes;
        bld.MakeCompound( holes );

        for( auto i : m_cutouts )
            bld.Add( holes, i );

        board = BRepAlgoAPI_Cut( board, holes );
    }

    // qwerty
    // XXX - push the board to the data structure
    m_assy->AddComponent( m_assy_label, board );

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


OUTLINE::OUTLINE()
{
    m_closed = false;
    return;
}


OUTLINE::~OUTLINE()
{
    return;
}


void OUTLINE::Clear()
{
    m_closed = false;
    m_curves.clear();
    return;
}


bool OUTLINE::AddSegment( const KICADCURVE& aCurve )
{
    if( m_closed )
        return false;

    if( m_curves.empty() )
    {
        m_curves.push_back( aCurve );

        if( CURVE_CIRCLE == aCurve.m_form )
            m_closed = true;

        return true;
    }

    if( CURVE_CIRCLE == aCurve.m_form )
        return false;

    // get the end points of the first curve
    double spx0, spy0;
    double epx0, epy0;
    getEndPoints( m_curves.front(), spx0, spy0, epx0, epy0 );

    // get the end points of the free curve
    double spx1, spy1;
    double epx1, epy1;
    getEndPoints( aCurve, spx1, spy1, epx1, epy1 );

    // check if the curve attaches to the front
    double dx, dy;
    dx = epx1 - spx0;
    dy = epy1 - spy0;

    if( dx * dx + dy * dy < MIN_LENGTH2 )
    {
        m_curves.push_front( aCurve );
        m_closed = testClosed( m_curves.front(), m_curves.back() );
        return true;
    }
    else
    {
        dx = spx1 - spx0;
        dy = spy1 - spy0;

        if( dx * dx + dy * dy < MIN_LENGTH2 )
        {
            KICADCURVE curve = aCurve;
            reverseCurve( curve );
            m_curves.push_front( curve );
            m_closed = testClosed( m_curves.front(), m_curves.back() );
            return true;
        }
    }

    // check if the curve attaches to the back
    getEndPoints( m_curves.back(), spx0, spy0, epx0, epy0 );
    dx = spx1 - epx0;
    dy = spy1 - epy0;

    if( dx * dx + dy * dy < MIN_LENGTH2 )
    {
        m_curves.push_back( aCurve );
        m_closed = testClosed( m_curves.front(), m_curves.back() );
        return true;
    }
    else
    {
        dx = epx1 - epx0;
        dy = epy1 - epy0;

        if( dx * dx + dy * dy < MIN_LENGTH2 )
        {
            KICADCURVE curve = aCurve;
            reverseCurve( curve );
            m_curves.push_back( curve );
            m_closed = testClosed( m_curves.front(), m_curves.back() );
            return true;
        }
    }

    // this curve is not an end segment of the current loop
    return false;
}

void printSegs( std::list< KICADCURVE >& curves )
{
    // XXX - TEST ONLY
    int idx = 0;
    for( auto i : curves )
    {
        switch( i.m_form )
        {
            case CURVE_LINE:
                std::cerr << "    [LINE][" << ++idx << "]\n";
                std::cerr << "        start(" << i.m_start.x << ", " << i.m_start.y << ")\n";
                std::cerr << "        end(" << i.m_end.x << ", " << i.m_end.y << ")\n";
                break;

            case CURVE_CIRCLE:
                std::cerr << "    [CIRCLE][" << ++idx << "]\n";
                std::cerr << "        center(" << i.m_start.x << ", " << i.m_start.y << ")\n";
                std::cerr << "         start(" << i.m_end.x << ", " << i.m_end.y << ")\n";
                std::cerr << "          rad: " << i.m_radius << "\n";
                break;

            case CURVE_ARC:
                std::cerr << "    [ARC][" << ++idx << "]\n";
                std::cerr << "        center(" << i.m_start.x << ", " << i.m_start.y << ")\n";
                std::cerr << "         start(" << i.m_end.x << ", " << i.m_end.y << ")\n";
                std::cerr << "           end(" << i.m_ep.x << ", " << i.m_ep.y << ")\n";
                std::cerr << "          rad: " << i.m_radius << "\n";
                std::cerr << "        angle: " << (i.m_angle * 180.0 / M_PI) << "\n";
                break;

            default:
                std::cerr << "    [UNKNOWN][" << ++idx << "]\n";
                break;
        }

    }
    return;
}

bool OUTLINE::MakeShape( TopoDS_Shape& aShape, double aThickness )
{
    if( !aShape.IsNull() )
        return false;   // there is already data in the shape object

    if( m_curves.empty() )
        return true;    // suceeded in doing nothing

    if( !m_closed )
        return false;   // the loop is not closed

    std::cerr << "** XXX: creating a shape with " << m_curves.size() << " segments\n";
    printSegs( m_curves );

    BRepBuilderAPI_MakeWire wire;
    DOUBLET lastPoint;
    getCurveEndPoint( m_curves.back(), lastPoint );

    for( auto i : m_curves )
    {
        if( !addEdge( wire, i, lastPoint ) )
            return false;
    }

    TopoDS_Face face = BRepBuilderAPI_MakeFace( wire );
    aShape = BRepPrimAPI_MakePrism( face, gp_Vec( 0, 0, aThickness ) );

    if( aShape.IsNull() )
    {
        std::ostringstream ostr;
        ostr << __FILE__ << ": " << __FUNCTION__ << ": " << __LINE__ << "\n";
        ostr << "  * failed to create a prismatic shape\n";
        wxLogMessage( "%s\n", ostr.str().c_str() );

        return false;
    }

    return true;
}


bool OUTLINE::addEdge( BRepBuilderAPI_MakeWire& aWire, KICADCURVE& aCurve, DOUBLET& aLastPoint )
{
    TopoDS_Edge edge;
    DOUBLET endPoint;
    getCurveEndPoint( aCurve, endPoint );

    // XXX - NOTE: for now we just use lines for everything
    switch( aCurve.m_form )
    {
        case CURVE_LINE:
            std::cerr << "** XXX: edge from (" << aLastPoint.x << ", " << aLastPoint.y;
            std::cerr << ") .. (" << endPoint.x << ", " << endPoint.y << ")\n";
            edge = BRepBuilderAPI_MakeEdge( gp_Pnt( aLastPoint.x, aLastPoint.y, 0.0 ),
                gp_Pnt( endPoint.x, endPoint.y, 0.0 ) );
            break;

        case CURVE_ARC:
            edge = BRepBuilderAPI_MakeEdge( gp_Pnt( aLastPoint.x, aLastPoint.y, 0.0 ),
                gp_Pnt( endPoint.x, endPoint.y, 0.0 ) );
            break;

        case CURVE_CIRCLE:
            edge = BRepBuilderAPI_MakeEdge( gp_Circ( gp_Ax2( gp_Pnt( aCurve.m_start.x, aCurve.m_start.y, 0.0 ),
                gp_Dir( 0.0, 0.0, 1.0 ) ), aCurve.m_radius ) );
            break;

        default:
            do
            {
                std::ostringstream ostr;
                ostr << __FILE__ << ": " << __FUNCTION__ << ": " << __LINE__ << "\n";
                ostr << "  * unsupported curve type: " << aCurve.m_form << "\n";
                wxLogMessage( "%s\n", ostr.str().c_str() );

                return false;
            } while( 0 );
    }

    if( edge.IsNull() )
        return false;

    aWire.Add( edge );
    aLastPoint = endPoint;

    return true;
}


bool OUTLINE::testClosed( KICADCURVE& aFrontCurve, KICADCURVE& aBackCurve )
{
    double spx0, spy0, epx0, epy0;
    getEndPoints( aFrontCurve, spx0, spy0, epx0, epy0 );
    double spx1, spy1, epx1, epy1;
    getEndPoints( aBackCurve, spx1, spy1, epx1, epy1 );

    double dx = epx1 - spx0;
    double dy = epy1 - spy0;
    double r = dx * dx + dy * dy;
    std::cerr << "** XXX: (r:" << r << ") ";

    if( r < MIN_LENGTH2 )
    {
        std::cerr << "[OK]\n";
        return true;
    }

    std::cerr << "[fail]\n";
    return false;
}

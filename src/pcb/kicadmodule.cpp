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

#include <wx/log.h>
#include <iostream>
#include <limits>
#include <sstream>

#include "3d_filename_resolver.h"
#include "sexpr/sexpr.h"
#include "kicadmodel.h"
#include "kicadmodule.h"
#include "kicadpad.h"
#include "kicadcurve.h"
#include "oce_utils.h"


KICADMODULE::KICADMODULE()
{
    m_side = LAYER_NONE;
    m_rotation = 0.0;

    return;
}


KICADMODULE::~KICADMODULE()
{
    for( auto i : m_pads )
        delete i;

    for( auto i : m_curves )
        delete i;

    for( auto i : m_models )
        delete i;

    return;
}


bool KICADMODULE::Read( SEXPR::SEXPR* aEntry )
{
    if( NULL == aEntry )
        return false;

    if( aEntry->IsList() )
    {
        size_t nc = aEntry->GetNumberOfChildren();
        SEXPR::SEXPR* child = aEntry->GetChild( 0 );
        std::string name = child->GetSymbol();

        if( name != "module" )
        {
            std::ostringstream ostr;
            ostr << "* BUG: module parser invoked for type '" << name << "'\n";
            wxLogMessage( "%s\n", ostr.str().c_str() );
            return false;
        }

        bool result = true;

        for( size_t i = 1; i < nc && result; ++i )
        {
            child = aEntry->GetChild( i );

            // skip the module name; due to the vagaries of the kicad
            // version of sexpr, the name may be a Symbol or a String
            if( i == 1 && ( child->IsSymbol() || child->IsString() ) )
                continue;

            if( !child->IsList() )
            {
                std::ostringstream ostr;
                ostr << "* corrupt module in PCB file\n";
                wxLogMessage( "%s\n", ostr.str().c_str() );
                return false;
            }

            std::string symname( child->GetChild( 0 )->GetSymbol() );

            if( symname == "layer" )
                result = result && parseLayer( child );
            else if( symname == "at" )
                result = result && parsePosition( child );
            else if( symname == "fp_text" )
                result = result && parseText( child );
            else if( symname == "fp_arc" )
                result = result && parseCurve( child, CURVE_ARC );
            else if( symname == "fp_line" )
                result = result && parseCurve( child, CURVE_LINE );
            else if( symname == "fp_circle" )
                result = result && parseCurve( child, CURVE_CIRCLE );
            else if( symname == "pad" )
                result = result && parsePad( child );
            else if( symname == "model" )
                result = result && parseModel( child );
        }

        return result;
    }

    std::ostringstream ostr;
    ostr << "* data is not a valid PCB module\n";
    wxLogMessage( "%s\n", ostr.str().c_str() );

    return false;
}


bool KICADMODULE::parseModel( SEXPR::SEXPR* data )
{
    KICADMODEL* mp = new KICADMODEL();

    if( !mp->Read( data ) )
    {
        delete mp;
        return false;
    }

    m_models.push_back( mp );
    return true;
}


bool KICADMODULE::parseCurve( SEXPR::SEXPR* data, CURVE_TYPE aCurveType )
{
    KICADCURVE* mp = new KICADCURVE();

    if( !mp->Read( data, aCurveType ) )
    {
        delete mp;
        return false;
    }

    // NOTE: for now we are only interested in glyphs on the outline layer
    if( LAYER_EDGE != mp->GetLayer() )
    {
        delete mp;
        return true;
    }

    m_curves.push_back( mp );
    return true;
}


bool KICADMODULE::parseLayer( SEXPR::SEXPR* data )
{
    SEXPR::SEXPR* val = data->GetChild( 1 );
    std::string layer;

    if( val->IsSymbol() )
        layer = val->GetSymbol();
    else if( val->IsString() )
        layer = val->GetString();
    else
    {
        std::ostringstream ostr;
        ostr << "* corrupt module in PCB file; layer cannot be parsed\n";
        wxLogMessage( "%s\n", ostr.str().c_str() );
        return false;
    }

    if( layer == "F.Cu" )
        m_side = LAYER_TOP;
    else if( layer == "B.Cu" )
        m_side = LAYER_BOTTOM;

    return true;
}


bool KICADMODULE::parsePosition( SEXPR::SEXPR* data )
{
    return Get2DPositionAndRotation( data, m_position, m_rotation );
}


bool KICADMODULE::parseText( SEXPR::SEXPR* data )
{
    // we're only interested in the Reference Designator
    if( data->GetNumberOfChildren() < 3 )
        return true;

    SEXPR::SEXPR* child = data->GetChild( 1 );
    std::string text;

    if( child->IsSymbol() )
        text = child->GetSymbol();
    else if( child->IsString() )
        text = child->GetString();

    if( text != "reference" )
        return true;

    child = data->GetChild( 2 );

    if( child->IsSymbol() )
        text = child->GetSymbol();
    else if( child->IsString() )
        text = child->GetString();

    m_refdes = text;
    return true;
}


bool KICADMODULE::parsePad( SEXPR::SEXPR* data )
{
    KICADPAD* mp = new KICADPAD();

    if( !mp->Read( data ) )
    {
        delete mp;
        return false;
    }

    // NOTE: for now we only accept thru-hole pads
    // for the MCAD description
    if( mp->IsThruHole() )
    {
        delete mp;
        return true;
    }

    m_pads.push_back( mp );
    return true;
}


bool KICADMODULE::ComposePCB( class PCBMODEL* aPCB, S3D_FILENAME_RESOLVER* resolver )
{
    // XXX - TO BE IMPLEMENTED
    // XXX - translate pads and curves to final position and append to PCB.
    /*
    std::vector< KICADPAD* >    m_pads;
    std::vector< KICADCURVE* >  m_curves;
     */

    double dlim = (double)std::numeric_limits< float >::epsilon();
    double vsin;
    double vcos;

    if( LAYER_TOP == m_side )
    {
        vsin = sin( m_rotation );
        vcos = cos( m_rotation );
    }
    else
    {
        vsin = sin( -m_rotation );
        vcos = cos( -m_rotation );
    }

    for( auto i : m_curves )
    {
        if( i->GetLayer() != LAYER_EDGE )
            continue;

        KICADCURVE lcurve = *i;

        if( LAYER_TOP == m_side )
        {
            lcurve.m_start.y = -lcurve.m_start.y;
            lcurve.m_end.y = -lcurve.m_end.y;
        }
        else
        {
            lcurve.m_angle = -lcurve.m_angle;
        }

        if( m_rotation < -dlim || m_rotation > dlim )
        {
            double x = lcurve.m_start.x * vcos - lcurve.m_start.y * vsin;
            double y = lcurve.m_start.x * vsin + lcurve.m_start.y * vcos;
            lcurve.m_start.x = x;
            lcurve.m_start.x = y;
            x = lcurve.m_end.x * vcos - lcurve.m_end.y * vsin;
            y = lcurve.m_end.x * vsin + lcurve.m_end.y * vcos;
            lcurve.m_end.x = x;
            lcurve.m_end.x = y;
        }

        lcurve.m_start.x += m_position.x;
        lcurve.m_start.y -= m_position.y;
        lcurve.m_end.x += m_position.x;
        lcurve.m_end.y -= m_position.y;

        aPCB->AddOutlineSegment( &lcurve );
    }

    for( auto i : m_pads )
    {
        if( !i->IsThruHole() )
            continue;

        KICADPAD lpad = *i;

        if( LAYER_TOP == m_side )
        {
            lpad.m_position.y = -lpad.m_position.y;

            if( lpad.m_drill.oval )
                lpad.m_rotation += m_rotation;
        }
        else
        {
            if( lpad.m_drill.oval )
                lpad.m_rotation += -(lpad.m_rotation + m_rotation);
        }

        if( m_rotation < -dlim || m_rotation > dlim )
        {
            double x = lpad.m_position.x * vcos - lpad.m_position.y * vsin;
            double y = lpad.m_position.x * vsin + lpad.m_position.y * vcos;
            lpad.m_position.x = x;
            lpad.m_position.x = y;
        }

        lpad.m_position.x += m_position.x;
        lpad.m_position.y -= m_position.y;

        aPCB->AddPadHole( &lpad );
    }

    for( auto i : m_models )
    {
        std::string fname( resolver->ResolvePath( i->m_modelname.c_str() ).ToUTF8() );
        aPCB->AddComponent( fname, m_refdes, LAYER_BOTTOM == m_side ? true : false,
            m_position, m_rotation, i->m_offset, i->m_rotation );
    }

    // XXX - TO BE IMPLEMENTED
    // XXX - ensure we only return true if model data was added
    return true;
}

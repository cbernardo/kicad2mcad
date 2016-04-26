/*
 * This program source code file is part kicad2mcad
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
#include <sstream>
#include "sexpr/sexpr.h"
#include "kicadmodule.h"

KICADMODULE::KICADMODULE()
{
    // XXX - TO BE IMPLEMENTED
    return;
}


KICADMODULE::~KICADMODULE()
{
    // XXX - TO BE IMPLEMENTED
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
                result = result && parseShape( child, SHAPE_ARC );
            else if( symname == "fp_line" )
                result = result && parseShape( child, SHAPE_LINE );
            else if( symname == "pf_circle" )
                result = result && parseShape( child, SHAPE_CIRCLE );
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
    // XXX - TO BE IMPLEMENTED
    return true;
}


bool KICADMODULE::parseShape( SEXPR::SEXPR* data, SHAPE_TYPE aShapeType )
{
    // XXX - TO BE IMPLEMENTED
    return true;
}


bool KICADMODULE::parseLayer( SEXPR::SEXPR* data )
{
    // XXX - TO BE IMPLEMENTED
    return true;
}


bool KICADMODULE::parsePosition( SEXPR::SEXPR* data )
{
    // XXX - TO BE IMPLEMENTED
    return true;
}


bool KICADMODULE::parseText( SEXPR::SEXPR* data )
{
    // XXX - TO BE IMPLEMENTED
    return true;
}


bool KICADMODULE::parsePad( SEXPR::SEXPR* data )
{
    // XXX - TO BE IMPLEMENTED
    return true;
}

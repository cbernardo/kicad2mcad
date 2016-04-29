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
#include <math.h>
#include "sexpr/sexpr.h"
#include "base.h"


bool Get2DPositionAndRotation( SEXPR::SEXPR* data, DOUBLET& aPosition, double& aRotation )
{
    if( NULL == data )
    {
        std::ostringstream ostr;
        ostr << "* invalid SEXPR pointer (NULL)";
        wxLogMessage( "%s\n", ostr.str().c_str() );
        return false;
    }

    // form: (at X Y {rot})
    const char bad_position[] = "* corrupt module in PCB file; invalid position";
    int nchild = data->GetNumberOfChildren();

    if( nchild < 3 )
    {
        std::ostringstream ostr;
        ostr << bad_position;
        wxLogMessage( "%s\n", ostr.str().c_str() );
        return false;
    }

    if( data->GetChild( 0 )->GetSymbol() != "at" )
    {
        std::ostringstream ostr;
        ostr << "* SEXPR item is not a position string";
        wxLogMessage( "%s\n", ostr.str().c_str() );
        return false;
    }

    SEXPR::SEXPR* child = data->GetChild( 1 );
    double x;

    if( child->IsDouble() )
        x = child->GetDouble();
    else if( child->IsInteger() )
        x = (double) child->GetInteger();
    else
    {
        std::ostringstream ostr;
        ostr << bad_position;
        wxLogMessage( "%s\n", ostr.str().c_str() );
        return false;
    }

    child = data->GetChild( 2 );
    double y;

    if( child->IsDouble() )
        y = child->GetDouble();
    else if( child->IsInteger() )
        y = (double) child->GetInteger();
    else
    {
        std::ostringstream ostr;
        ostr << bad_position;
        wxLogMessage( "%s\n", ostr.str().c_str() );
        return false;
    }

    aPosition.x = x;
    aPosition.y = y;

    if( nchild == 3 )
        return true;

    child = data->GetChild( 3 );
    double angle = 0.0;

    if( child->IsDouble() )
        angle = child->GetDouble();
    else if( child->IsInteger() )
        angle = (double) child->GetInteger();
    else
    {
        std::ostringstream ostr;
        ostr << bad_position;
        wxLogMessage( "%s\n", ostr.str().c_str() );
        return false;
    }

    while( angle >= 360.0 )
        angle -= 360.0;

    while( angle <= -360.0 )
        angle += 360.0;

    aRotation = (angle / 180.0) * M_PI;

    return true;
}

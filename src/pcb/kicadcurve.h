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

/**
 * @file kicadcurve.h
 * declares the Curve (glyph) object.
 */

#ifndef KICADCURVE_H
#define KICADCURVE_H

#include <string>
#include <vector>
#include "base.h"


class KICADCURVE
{
private:
    CURVE_TYPE m_form;  // form of curve: line, arc, circle
    DOUBLET    m_start; // start point of line or center for arc and circle
    DOUBLET    m_end;   // end point of line, first point on arc or circle
    double     m_angle; // subtended angle of arc
    LAYERS     m_layer; // layer of the glyph

public:
    KICADCURVE();
    virtual ~KICADCURVE();

    bool Read( SEXPR::SEXPR* aEntry, CURVE_TYPE aCurveType );

    LAYERS GetLayer()
    {
        return m_layer;
    }
};

#endif  // KICADCURVE_H

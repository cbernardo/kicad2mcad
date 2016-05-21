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

/**
 * @file kicadpcb.h
 * declares the main PCB object
 */

#ifndef KICADPCB_H
#define KICADPCB_H

#include <wx/string.h>
#include <string>
#include <vector>
#include "3d_filename_resolver.h"
#include "base.h"

namespace SEXPR
{
    class SEXPR;
}

class KICADMODULE;
class KICADCURVE;
class PCBMODEL;

class KICADPCB
{
private:
    S3D_FILENAME_RESOLVER m_resolver;
    std::string m_filename;
    PCBMODEL*   m_pcb;

    // PCB parameters/entities
    double                      m_thickness;
    std::vector< KICADMODULE* > m_modules;
    std::vector< KICADCURVE* >  m_curves;

    bool parsePCB( SEXPR::SEXPR* data );
    bool parseGeneral( SEXPR::SEXPR* data );
    bool parseModule( SEXPR::SEXPR* data );
    bool parseCurve( SEXPR::SEXPR* data, CURVE_TYPE aCurveType );

public:
    KICADPCB();
    virtual ~KICADPCB();

    bool ReadFile( const wxString& aFileName );
    bool ComposePCB();
    void SetLogging( bool aUseLog );
    bool WriteSTEP( const wxString& aFileName, bool aOverwrite );
    bool WriteIGES( const wxString& aFileName, bool aOverwrite );
};


#endif  // KICADPCB_H

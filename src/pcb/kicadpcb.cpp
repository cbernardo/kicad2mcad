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

#include "kicadpcb.h"

#include <wx/utils.h>
#include <wx/filename.h>
#include <wx/stdpaths.h>
#include <iostream>
#include "3d_filename_resolver.h"


/*
 * GetKicadConfigPath() is taken from KiCad's common.cpp source:
 * Copyright (C) 2014-2015 Jean-Pierre Charras, jp.charras at wanadoo.fr
 * Copyright (C) 2008-2015 Wayne Stambaugh <stambaughw@verizon.net>
 * Copyright (C) 1992-2015 KiCad Developers
 */
static wxString GetKicadConfigPath()
{
    wxFileName cfgpath;

    // From the wxWidgets wxStandardPaths::GetUserConfigDir() help:
    //      Unix: ~ (the home directory)
    //      Windows: "C:\Documents and Settings\username\Application Data"
    //      Mac: ~/Library/Preferences
    cfgpath.AssignDir( wxStandardPaths::Get().GetUserConfigDir() );

#if !defined( __WINDOWS__ ) && !defined( __WXMAC__ )
    wxString envstr;

    if( !wxGetEnv( wxT( "XDG_CONFIG_HOME" ), &envstr ) || envstr.IsEmpty() )
    {
        // XDG_CONFIG_HOME is not set, so use the fallback
        cfgpath.AppendDir( wxT( ".config" ) );
    }
    else
    {
        // Override the assignment above with XDG_CONFIG_HOME
        cfgpath.AssignDir( envstr );
    }
#endif

    cfgpath.AppendDir( wxT( "kicad" ) );

    if( !cfgpath.DirExists() )
    {
        cfgpath.Mkdir( wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL );
    }

    return cfgpath.GetPath();
}


KICADPCB::KICADPCB()
{
    wxFileName cfgdir( GetKicadConfigPath(), "" );
    cfgdir.AppendDir( "3d" );
    m_resolver.Set3DConfigDir( cfgdir.GetPath() );

    return;
}


KICADPCB::~KICADPCB()
{
    return;
}


bool KICADPCB::ReadFile( const wxString& aFileName )
{
    // XXX - TO BE IMPLEMENTED
    return false;
}


void KICADPCB::SetLogging( bool aUseLog )
{
    // XXX - TO BE IMPLEMENTED
    return;
}


bool KICADPCB::WriteSTEP( const wxString& aFileName, bool aOverwrite )
{
    // XXX - TO BE IMPLEMENTED
    return false;
}


bool KICADPCB::WriteIGES( const wxString& aFileName, bool aOverwrite )
{
    // XXX - TO BE IMPLEMENTED
    return false;
}

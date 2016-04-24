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
 * @file kicadmodule.h
 * declares the PCB Component object.
 */

#ifndef KICADMODULE_H
#define KICADMODULE_H

class SEXPR::SEXPR;
class KICADPCB;

class KICADMODULE
{
private:
    KICADPCB* m_parent;

public:
    KICADMODULE( KICADPCB* aParent );
    virtual ~KICADMODULE();

    bool Read( SEXPR::SEXPR* aEntry );

};

#endif  // KICADMODULE_H

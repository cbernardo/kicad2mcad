include_directories( pcb ${CMAKE_CURRENT_SOURCE_DIR} )

add_executable( mtest
    kicad2mcad.cpp
    pcb/3d_filename_resolver.cpp
    pcb/kicadpcb.cpp
    pcb/kicadmodule.cpp
    sexpr/sexpr.cpp
    sexpr/sexpr_parser.cpp
)

target_link_libraries( mtest ${wxWidgets_LIBRARIES} )

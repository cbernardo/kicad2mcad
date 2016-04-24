include_directories( pcb )

add_executable( test
    pcb/main.cpp
    pcb/3d_filename_resolver.cpp
    pcb/kicadpcb.cpp
)

target_link_libraries( test ${wxWidgets_LIBRARIES} )

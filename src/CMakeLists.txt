add_executable(main)

target_include_directories(
    main
  PUBLIC 
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>"
    ${X11_INCLUDE_DIR}
)

target_link_libraries(main PUBLIC ${X11_LIBRARIES})

target_sources(
    main
  PRIVATE 
    error.c
    gui.c
    you.c
)
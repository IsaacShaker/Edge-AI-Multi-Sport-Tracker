# embed_html.cmake
# Reads an HTML file and writes a C++ header that exposes it as a
# null-terminated const char* via a raw string literal.
#
# Called by CMakeLists.txt add_custom_command:
#   cmake -DINPUT=<html_file> -DOUTPUT=<header_file> -P embed_html.cmake

if(NOT INPUT OR NOT OUTPUT)
    message(FATAL_ERROR "Usage: cmake -DINPUT=<html> -DOUTPUT=<header> -P embed_html.cmake")
endif()

# Ensure the output directory exists before writing the file.
get_filename_component(OUTPUT_DIR "${OUTPUT}" DIRECTORY)
file(MAKE_DIRECTORY "${OUTPUT_DIR}")

file(READ "${INPUT}" HTML_CONTENT)

file(WRITE "${OUTPUT}"
"// Auto-generated from ${INPUT} — do not edit manually.
// Re-generated automatically whenever web/index.html changes.
#pragma once
namespace tracker {
inline const char* kIndexHtml() {
    static const char* html = R\"HTMLDOC(
${HTML_CONTENT})HTMLDOC\";
    return html;
}
} // namespace tracker
")

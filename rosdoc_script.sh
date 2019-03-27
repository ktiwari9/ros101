#!/bin/bash

# # generate ros project_name list
project_names=()
while read i; do
    proj=${i#$(pwd)/}/
    project_names+=($proj)
done < <(find `pwd` -iname "package.xml" -printf "%h\n" | sort -u)

# generate doc for each ros projects
for project_name in ${project_names[@]};do
    printf "\nProject $project_name :\n"
    rosdoc_lite $project_name -o $project_name\doc
    mkdir -p .public/$project_name\doc/html
    cp -rv $project_name\doc/html/* .public/$project_name\doc/html/
    # add home button
    sed -i 's_class="contents">_class="contents"><a href="../../../../index.html">Back to tutorial page</a>_1' .public/$project_name\doc/html/index.html

    # add project tags to the main .doxygen/Doxyfile
    #sed -i '/^TAGFILES/ s/$/ "replacename\/doc\/replacename.tag\ =\ replacename\/doc\/html"/' .doxygen/Doxyfile
    #sed -i 's|replacename|'${project_name///}'|g' .doxygen/Doxyfile

    # add project tags to docs/package.md
    echo -e "[$project_name package](${project_name}doc/html/index.html)" >> docs/packages.md
    echo -e >> docs/packages.md
done

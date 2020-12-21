Documentation {#documentation}
=============

[TOC]

# General

The Autoware.Auto documentation is created with the `doxygen` tool based on text files written in
markdown; see [markdown in doxygen](https://www.doxygen.nl/manual/markdown.html) for details.

Lines in markdown shall in general be limited to 100 characters with exceptions when that's
impractical; e.g.

- long links
- source code

To ease code review, it is recommended that each sentence start on a new line.
This doesn't break paragraphs unless a blank line is included.

## Images

Illustrations and screenshots are great to make a point and can save a lot of text. Place an image
into the `docs/images` folder and refer to it as

    @image html images/foo.png "image caption"

This way of including ensures that doxygen fails if it cannot find the image.

## Integrating a new document

Add an achor for a new document and use the anchors to link to other parts of the documentation. A
minimal example:

```md
New documentation {#new-documentation}
=================

[TOC]

# First section

# Second section
```

Make sure the document is included by an appropriate `index.md` such that it appears at the desired
location; e.g.,

```md
- @subpage new-documentation
```

## Rendering

Verify the documentation builds without errors and appears as desired by building locally first:

    docs/.doxygen/build.py

At the end of the long output of this command, the entry point is displayed. Open in a web browser

    Documentation has been built in: /home/user/AutowareAuto/docs/_build/html/index.html

As a final check before merging, validate that the documentation built in CI is correct by browsing
the artifacts of the *docs* stage of the merge request's build job. First open the pipeline of the
merge request on gitlab, then select the docs job:

@image html images/documentation-build-ci.png "Select the docs build job"

Finally browse the artifact and select the html file(s) modified in the merge request.

@image html images/documentation-browse.png "Browse the documentation artifact"

# Documenting a package

Packages shall be accompanied with a design document written in markdown; e.g. in
`pkg_foo/design/pkg_foo-design.md`. There can be several files in `design/` if needed. The purpose
of the design document is to describe the intended behavior of a component. Serving as an entry
point for users unfamiliar with that component, it should explain at a sufficient abstraction level.

# Documenting source code

All C++ code that is to be consumed by someone else should be declared in header files and should
come with doxygen comments including classes, structs, methods, members etc.

Use the imperative to describe each entity. Finish each section with a period `.`.

For example:

```c++
/// Analyse input for consistency.
/// @param input The input, assumed `>= 1.2`.
/// \throws A `std::invalid_argument` if invalid.
explicit class Foo(double input);

/// \return `true` if the input is valid.
bool valid() const;
```

## Comments inside code

Add comments where the code is not self-explanatory. When adding comments, think about
renaming/restructuring to make the code self-explanatory.

Don't add this comment as it doesn't provide useful information

```c++
// distance
double distance = 3.2;
```

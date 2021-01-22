Writing documentation {#writing-documentation}
=====================

[TOC]

# General

The Autoware.Auto documentation is created with the `doxygen` tool based on text files written in
markdown; see [markdown in doxygen](https://www.doxygen.nl/manual/markdown.html) for details.

## Rendering {#documentation-rendering}

Verify the documentation builds without errors and appears as desired by building locally first:

    docs/.doxygen/build.py

At the end of the long output of this command, the entry point is displayed. Open in a web browser

    Documentation has been built in: /home/user/AutowareAuto/docs/_build/html/index.html

### In CI

As a final check before merging, validate that the documentation built in CI is correct by browsing
the artifacts of the *docs* stage of the merge request's build job. The URL will be similar to the
following but the build job ID has to be modified

https://autowarefoundation.gitlab.io/-/autoware.auto/AutowareAuto/-/jobs/BUILD_JOB_ID/artifacts/docs/_build/html/index.html

To select the docs with a mouse, first open the pipeline of the merge request on GitLab, then select the docs job:

@image html images/documentation-build-ci.png "Select the docs build job"

Finally browse the artifact and select the html file(s) modified in the merge request.

@image html images/documentation-browse.png "Browse the documentation artifact"

## Markdown guidelines

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

Add an anchor for a new document; e.g.

    #new-document

after the title and use the anchor to link to other parts of the documentation. A minimal example:

```md
New document {#new-document}
=================

[TOC]

# First section

# Second section
```

And within another `foo.md`, refer to the new document with:

```md
@ref new-document "See the new document"
```

Make sure the document is included by an appropriate `index.md` such that it appears at the desired
location; e.g.,

```md
- @subpage new-document
```

## Links from the outside

Documents that link to a section in the doxygen output also need an anchor for that URL to be stable with respect to
documentation updates. To that end, the anchor should have a prefix that's unique to the page in which the section
lives.

**Recommended**

Inside `document.md`:

```md
[TOC]

# Foo {#document-foo}
```

then the URL is https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/document.html#document-foo

**Discouraged**

Inside `document.md`:

```md
[TOC]

# Foo
```

then the URL could be e.g. https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/document.html#autotoc_md21

Notice the `autotoc_md21` at the end of the URL. Doxygen increments a counter to automatically
create URLs for sections without an anchor. If the section `Foo` is moved or another section added somewhere else, the URL may become invalid.

# Documenting a package {#documentation-package}

Packages shall be accompanied with a design document written in markdown; e.g. in
`pkg_foo/design/pkg_foo-design.md`. There can be several files in `design/` if needed. The purpose
of the design document is to describe the intended behavior of a component. Serving as an entry
point for users unfamiliar with that component, it should explain at a sufficient abstraction level.

# Documenting source code {#documentation-source-code}

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

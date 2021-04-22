## Description
<!-- What is the high-level purpose of this merge request? Link to existing issue -->

## Notes for reviewer
<!-- Items in addition to the checklist below that the reviewer should pay special attention to. How to test the code... -->

## Pre-review checklist for the author before submitting for review

Every developer is encouraged to be familiar with our [contributor guidelines](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/contributor-guidelines.html).

1. MR formalities
   1. [ ] "WIP" or "Draft" removed from the MR title
   1. [ ] MR title and description help a friendly human understand the problem solved
   1. [ ] MR has a link to the original issue in the description, if it exists
   1. [ ] If the source branch is on a fork, MR is configured to *allow commits from developers with access to push to the target branch*
   1. [ ] Sensible notes for the reviewer added to the section above to facilitate review
   1. [ ] Target branch set correctly. Default: `master`
   1. [ ] MR assigned to a capable reviewer. Default: JWhitleyWork
   1. [ ] Splitting the MR into smaller, easier-to-review merge requests was considered
1. Code and tests
   1. [ ] Code is properly [formatted](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/contributor-guidelines.html#contributors-guidelines-formatting)
   1. [ ] [Tests](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/contributor-guidelines.html#contributors-guidelines-run-tests) affected by new code pass locally
   1. [ ] Reasonable [coverage with unit tests](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/contributor-guidelines.html#contributors-guidelines-coverage) of 90+%; else create a follow-up ticket
   1. [ ] Review any `// TODO` item added in the MR that can be addressed without the reviewer's help
1. Documentation
   1. [ ] Any new and modified code has accurate [doxygen documentation](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/documentation.html#documentation-source-code)
   1. [ ] Any diagrams are committed

## Checklist for the reviewer

**Only the reviewer is allowed to make changes in this section!**

Items not applicable to this MR are crossed out by the reviewer.

- [ ] For new nodes, the [checklist](#new-node) is expanded and reviewed
- [ ] For a new package, the [checklist](#new-package) is expanded and reviewed
- [ ] Reviewer crossed out non-applicable items

### Checklist

If the MR provides an improvement, don't hesitate to add a :thumbsup: emoji for a neat line of code or a "Thanks for implementing this" comment. This will reward the MR author and prevent the review from being only about what still needs to be improved.

Mark all the items that are done.
<details>
<summary markdown="span"><a name="general">Checklist for development</a></summary>

1. Basic checks
   1. [ ] The MR title describes what is being done on the ticket
   1. [ ] All functional code written in C++14, tooling code may be written in Python 3.5+ or Bash
   1. [ ] The first commit has a proper [commit message](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/contributor-guidelines.html#contributors-guidelines-committing) to be used as a basis for the squashed commit created at the very end; e.g. `[#928] Fix foo in bar`
1. Code correctness
   1. [ ] The problem/feature is solved (reproducibly)
   1. [ ] The solution is performant enough for the use case in mind
1. Open work
   1. [ ] Any added source-code comment about future work refers to a follow-up GitLab issue explicitly; e.g., `// TODO #551 refactor code below`
1. Documentation
   1. [ ] New classes, methods, functions in headers are documented with doxygen-style comments
   1. [ ] If implementation (of a function...) is modified, the doxygen documentation is updated accordingly
   1. [ ] The [design article](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/documentation.html#documentation-package) is updated with the implementation
   1. [ ] Drawings are created when needed and committed to `git`
   1. [ ] Modified files have a license that is [compatible](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/CONTRIBUTING.md) with AutowareAuto
1. Testing
   1. [ ] Code coverage with unit tests does not decrease. Aim for coverage with unit tests of 90+%; else create a follow-up ticket
   1. [ ] Unit tests make sense and cover the business logic and error cases
   1. [ ] Integration tests are sensible and not flaky

</details>

<details>
  <summary markdown="span"><a name="new-node">Checklist for new ROS2 nodes</a></summary>

1. [ ] Every node is [registered as a component](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/contributor-guidelines.html#contributors-guidelines-components)
1. [ ] The [naming conventions](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/autoware-common-naming-guidelines.html) are followed
1. [ ] At least the basic [launch component test](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/integration-testing.html#integration-testing-component-test) is included

</details>

<details>
  <summary markdown="span"><a name="new-package">Checklist for new package</a></summary>

1. Structure
   1. [ ] The package name and organization into files is sensible
   1. [ ] The files have a license header as per [CONTRIBUTING.md](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/CONTRIBUTING.md)
   1. [ ] Core functionality is separated from the ROS2-specific part where reasonable
   1. [ ] There is a design document that explains the package at a high level
   1. [ ] All dependencies are explicitly included in [`package.xml`](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/contributor-guidelines.html#contributors-guidelines-minimal-package-xml-example) with the proper `<*depend>` declaration

When starting from scratch, new packages should be created with the [`autoware_auto_create_pkg`](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/autoware_auto_create_pkg-package-design.html) macro and they will automatically satisfy the following criteria.

1. If an existing package is added to `AutowareAuto`, it should match the output of `autoware_auto_create_pkg` regarding
   1. [ ] calling `autoware_set_compile_options` for each compiled target
   1. [ ] same set of linters
   1. [ ] visibility control
   1. [ ] finding build dependencies in [`cmake`](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/contributor-guidelines.html#contributors-guidelines-minimal-cmake-example) with `ament_auto_find_build_dependencies()`
   1. [ ] installing with `ament_auto_package()`
 </details>

## Post-review checklist for the author

After receiving approval:

1. [ ] [Rendered documentation](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/documentation.html#documentation-rendering) looks as expected
1. [ ] All checkboxes in the MR checklist are checked or crossed out. Syntax example: `1. [ ] ~~this item crossed out~~`
1. [ ] Developers were informed about breaking changes on slack in the _committers-autoware-auto_ channel
1. [ ] Assign MR to maintainer with sufficient rights to merge. Default: JWhitleyWork

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package leg_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.9 (2015-09-01)
------------------
* Fix handling of scans with negative angle increment
  For laser sensors which are mounted upside down. Old code did not generate any clusters in that case.
* Contributors: Timm Linder

1.0.8 (2014-12-10)
------------------
* cleanup formatting with astyle (supersedes `#18 <https://github.com/wg-perception/people/issues/18>`_)
* Contributors: Dan Lazewatsky

1.0.4 (2014-07-09)
------------------
* Merging leg_detector into people
* Contributors: David Lu!!

1.0.3 (2014-03-01)
------------------

1.0.2 (2014-02-28)
------------------

1.0.1 (2014-02-27)
------------------
* height_tracker --> people_experimental
  git-svn-id: https://code.ros.org/svn/wg-ros-pkg/trunk/stacks/people@40194 7275ad9f-c29b-430a-bdc5-66f4b3af1622
* Added platform tags for Ubuntu 9.04, 9.10, and 10.04.
  git-svn-id: https://code.ros.org/svn/wg-ros-pkg/trunk/stacks/people@36945 7275ad9f-c29b-430a-bdc5-66f4b3af1622
* Unblacklisting
  git-svn-id: https://code.ros.org/svn/wg-ros-pkg/trunk/stacks/people@33043 7275ad9f-c29b-430a-bdc5-66f4b3af1622
* removed a number of dependencies
  git-svn-id: https://code.ros.org/svn/wg-ros-pkg/trunk/stacks/people@32910 7275ad9f-c29b-430a-bdc5-66f4b3af1622
* git-svn-id: https://code.ros.org/svn/wg-ros-pkg/trunk/stacks/people@32909 7275ad9f-c29b-430a-bdc5-66f4b3af1622
* people: blacklisting packages due to deprecated deps
  git-svn-id: https://code.ros.org/svn/wg-ros-pkg/trunk/stacks/people@32439 7275ad9f-c29b-430a-bdc5-66f4b3af1622
* leg detector from people_package --> leg_detector
  git-svn-id: https://code.ros.org/svn/wg-ros-pkg/trunk/stacks/people@32067 7275ad9f-c29b-430a-bdc5-66f4b3af1622
* moved the filter into the filter package
  git-svn-id: https://code.ros.org/svn/wg-ros-pkg/trunk/stacks/people@31998 7275ad9f-c29b-430a-bdc5-66f4b3af1622
* empty packages that will eventually hold the stuff in people_package
  git-svn-id: https://code.ros.org/svn/wg-ros-pkg/trunk/stacks/people@31910 7275ad9f-c29b-430a-bdc5-66f4b3af1622
* Contributors: Brian Gerkey, Caroline Pantofaru, Ken Conley

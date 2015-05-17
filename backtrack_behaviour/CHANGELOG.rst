^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package backtrack_behaviour
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.17 (2015-05-17)
-------------------

0.0.16 (2015-05-13)
-------------------
* Changed to utf8 character encoding and added commas
* Added localization for backtracking and help screens and speech
* Contributors: Nils Bore

0.0.15 (2015-04-28)
-------------------

0.0.14 (2015-04-22)
-------------------

0.0.13 (2015-04-10)
-------------------
* Extended the wait time a bit
* Corrected message assignment
* Changed to check if correct plan with service instead
* Moved speaking from monitored nav states to backtrack, with check if it got a global plan
* Contributors: Nils Bore, Rares Ambrus

0.0.12 (2015-03-24)
-------------------

0.0.10 (2014-11-23)
-------------------
* Added a try catch to reconfiguring move_base parameters
* Contributors: Nils Bore

0.0.9 (2014-11-21)
------------------

0.0.8 (2014-11-21)
------------------

0.0.7 (2014-11-21)
------------------
* updated changelogs
* Forgot to change the speed of going back to 0 as well
* Return success if backtrack moves at all
* Fixed bug
* Added output if no head camera
* Made the head turn the other way and not at all if no head_obstacle_cloud in sources
* Contributors: Marc Hanheide, Nils Bore

* Forgot to change the speed of going back to 0 as well
* Return success if backtrack moves at all
* Fixed bug
* Added output if no head camera
* Made the head turn the other way and not at all if no head_obstacle_cloud in sources
* Contributors: Nils Bore

* Forgot to change the speed of going back to 0 as well
* Return success if backtrack moves at all
* Fixed bug
* Added output if no head camera
* Made the head turn the other way and not at all if no head_obstacle_cloud in sources
* Contributors: Nils Bore

* Forgot to change the speed of going back to 0 as well
* Return success if backtrack moves at all
* Fixed bug
* Added output if no head camera
* Made the head turn the other way and not at all if no head_obstacle_cloud in sources
* Contributors: Nils Bore

* Forgot to change the speed of going back to 0 as well
* Return success if backtrack moves at all
* Fixed bug
* Added output if no head camera
* Made the head turn the other way and not at all if no head_obstacle_cloud in sources
* Contributors: Nils Bore

0.0.6 (2014-11-19)
------------------

0.0.5 (2014-11-18)
------------------

0.0.4 (2014-11-14)
------------------

0.0.3 (2014-11-11)
------------------
* Added cmake_modules as a dependency to get Eigen
* Contributors: Nils Bore

0.0.2 (2014-11-10)
------------------

0.0.1 (2014-11-10)
------------------
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Changed to the correct topic
* backtrack server now waits for SetPTUState and move_base
* Removed unnecessary dependencies and corrected an include bug
* Added the correct paths in backtrack_server.py
* Fixed the message namespaces
* Now the backtrack_server.py is executable after installing
* Added install targets for backtrack_behaviour package
* Launching the correct nodes in the launch file
* Got it to compile
* Integrated previous_positions_service and republish_pointcloud_service into backtrack_behaviour package
* moving backtrack related packages here
* Contributors: Bruno Lacerda, Nils Bore

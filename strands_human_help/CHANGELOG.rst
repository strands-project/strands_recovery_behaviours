^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_human_help
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2018-10-30)
------------------
* Update help_screen.py
* Contributors: Bruno Lacerda

0.0.19 (2017-01-11)
-------------------
* small bug fix
* use modal dialog
* Contributors: Bruno Lacerda

0.0.18 (2016-11-03)
-------------------

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
* only complain about lack of proper help when help was offered
* first stuff to allow recover states to be deactivated
* Contributors: Bruno Lacerda

0.0.12 (2015-03-24)
-------------------

0.0.10 (2014-11-23)
-------------------
* Changing help speech to explicitely tell people to look to the robot's screen
* bug fixes for twitter help
* adding twitter help - always for magnetic strip, once when nav or bumper fails 20 times in a row
* Contributors: Bruno Lacerda

0.0.9 (2014-11-21)
------------------

0.0.8 (2014-11-21)
------------------

0.0.7 (2014-11-21)
------------------
* updated changelogs
* magnetic strip monitor and recovery
  just asks to call a robot handler and puts a message on screen
  to restart the navigation there needss to be an explicit service call to /reset_barrier_stop done by one of us
* proper warning message
* stop waiting for mary as it might block the whole state machine
* Contributors: Bruno Lacerda, Marc Hanheide

* magnetic strip monitor and recovery
  just asks to call a robot handler and puts a message on screen
  to restart the navigation there needss to be an explicit service call to /reset_barrier_stop done by one of us
* proper warning message
* stop waiting for mary as it might block the whole state machine
* Contributors: Bruno Lacerda

0.0.6 (2014-11-19)
------------------
* say correct sentence after help fail
* Contributors: Bruno Lacerda

0.0.5 (2014-11-18)
------------------
* keeping nodes printing run dependencies that are missing
* Contributors: Bruno Lacerda

0.0.4 (2014-11-14)
------------------

0.0.3 (2014-11-11)
------------------
* Merge pull request `#15 <https://github.com/strands-project/strands_recovery_behaviours/issues/15>`_ from BFALacerda/hydro-devel
  small bug fix
* update maintainer name in package.xml
* Contributors: BFALacerda, Bruno Lacerda

0.0.2 (2014-11-10)
------------------

0.0.1 (2014-11-10)
------------------
* only say thank you when you're helped
* merging nav help speech and nav help screen to a single package
  total rewrite of the classes in order to make their addition and removal to monitored nav similar to the recovery behaviours
* Contributors: Bruno Lacerda

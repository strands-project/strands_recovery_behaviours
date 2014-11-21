^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_human_help
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

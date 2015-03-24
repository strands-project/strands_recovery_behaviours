^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_monitored_nav_states
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.12 (2015-03-24)
-------------------

0.0.10 (2014-11-23)
-------------------
* Making sure free run is turned off after help is done
* adding simple "pause and the rety nav" recovery state
* removing clear costmaps recovery as move base seems to not be robust in terms of costmap reloading
* increase default of max bumper recoveries to very large number. it will be removed later because it doesnt make sense
* robot speaks to warn he is going to backtrack
* Contributors: Bruno Lacerda

0.0.9 (2014-11-21)
------------------
* fixing typo
* catching clear costmap exceptions
* Contributors: Bruno

0.0.8 (2014-11-21)
------------------

0.0.7 (2014-11-21)
------------------
* updated changelogs
* changing default mongo loging back to true
* minor code clean
* magnetic strip monitor and recovery
  just asks to call a robot handler and puts a message on screen
  to restart the navigation there needss to be an explicit service call to /reset_barrier_stop done by one of us
* reducing time waited before clearing the costmaps
* making sure states preempt
* bug fix on clear costmaps recovery
* making sure bumper monitor does not try to restart motors after finding a barrier
* adding force clear costmap state;
  all nav recovery states put in the same file
* stop using max recover attempts for nav as it always output to topological nav anyway
* Contributors: Bruno Lacerda, Marc Hanheide

* changing default mongo loging back to true
* minor code clean
* magnetic strip monitor and recovery
  just asks to call a robot handler and puts a message on screen
  to restart the navigation there needss to be an explicit service call to /reset_barrier_stop done by one of us
* reducing time waited before clearing the costmaps
* making sure states preempt
* bug fix on clear costmaps recovery
* making sure bumper monitor does not try to restart motors after finding a barrier
* adding force clear costmap state;
  all nav recovery states put in the same file
* stop using max recover attempts for nav as it always output to topological nav anyway
* Contributors: Bruno Lacerda

* changing default mongo loging back to true
* minor code clean
* magnetic strip monitor and recovery
  just asks to call a robot handler and puts a message on screen
  to restart the navigation there needss to be an explicit service call to /reset_barrier_stop done by one of us
* reducing time waited before clearing the costmaps
* making sure states preempt
* bug fix on clear costmaps recovery
* making sure bumper monitor does not try to restart motors after finding a barrier
* adding force clear costmap state;
  all nav recovery states put in the same file
* stop using max recover attempts for nav as it always output to topological nav anyway
* Contributors: Bruno Lacerda

* changing default mongo loging back to true
* minor code clean
* magnetic strip monitor and recovery
  just asks to call a robot handler and puts a message on screen
  to restart the navigation there needss to be an explicit service call to /reset_barrier_stop done by one of us
* reducing time waited before clearing the costmaps
* making sure states preempt
* bug fix on clear costmaps recovery
* making sure bumper monitor does not try to restart motors after finding a barrier
* adding force clear costmap state;
  all nav recovery states put in the same file
* stop using max recover attempts for nav as it always output to topological nav anyway
* Contributors: Bruno Lacerda

* changing default mongo loging back to true
* minor code clean
* magnetic strip monitor and recovery
  just asks to call a robot handler and puts a message on screen
  to restart the navigation there needss to be an explicit service call to /reset_barrier_stop done by one of us
* reducing time waited before clearing the costmaps
* making sure states preempt
* bug fix on clear costmaps recovery
* making sure bumper monitor does not try to restart motors after finding a barrier
* adding force clear costmap state;
  all nav recovery states put in the same file
* stop using max recover attempts for nav as it always output to topological nav anyway
* Contributors: Bruno Lacerda

0.0.6 (2014-11-19)
------------------

0.0.5 (2014-11-18)
------------------
* keeping nodes printing run dependencies that are missing
* wait a bit more to ask for help
* changing default backtrack tries to 2
* moving service pause to monitored_navigation
* editing human help service
* debugging bumper recovery
* solving ask help service call bug
* Contributors: Bruno Lacerda

0.0.4 (2014-11-14)
------------------
* only logs to db is explicitly told to do it
* Contributors: Bruno Lacerda

0.0.3 (2014-11-11)
------------------
* small bug fix
* Contributors: Bruno Lacerda

0.0.2 (2014-11-10)
------------------

0.0.1 (2014-11-10)
------------------
* using new AskHelp srv definition
* renaming smach recoveries package
* Contributors: Bruno Lacerda

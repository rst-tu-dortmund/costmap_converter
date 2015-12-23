^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package costmap_converter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* The argument list of the initialize method requires a nodehandle from now on. This facilitates the handling of parameter namespaces for multiple instantiations of the plugin.
* This change is pushed immediately as a single release to avoid API breaks (since version 0.0.2 is not on the official repos up to now).

0.0.2 (2015-12-22)
------------------
* Added a plugin for converting the costmap to lines using ransac

0.0.1 (2015-12-21)
------------------
* First release of the package including a pluginlib interface, two plugins (costmap to polygons and costmap to lines) and a standalone conversion node.


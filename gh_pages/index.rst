.. SphinxTest documentation master file, created by
   sphinx-quickstart on Tue Oct  3 11:09:13 2017.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

======================================
Welcome to the Tesseract Planning wiki
======================================

This include packages related to both motion and process planning for the Tesseract Motion Planning Environment.


Tesseract Planning Packages
---------------------------

* **tesseract_motion_planners** – This package contains a common interface for Planners and includes implementation for OMPL, TrajOpt, TrajOpt IFOPT and Descartes.
* **tesseract_process_managers** – This package contains a common interface for Process Planning and includes implementation for a wide variaty of process found industrial automation like painting, griding, welding, pick and place and more.
* **tesseract_time_parameterization** – This package contains a time parameterization algorithms and includes iteritive spline.

Related Repositories
--------------------

* `Tesseract<https://github.com/ros-industrial-consortium/tesseract>`_
* `Tesseract Python<https://github.com/ros-industrial-consortium/tesseract_python>`_
* `Tesseract ROS<https://github.com/ros-industrial-consortium/tesseract_python>`_

.. Warning:: These packages are under heavy development and are subject to change.

Packages
--------

.. toctree::
   :maxdepth: 1

   tesseract_motion_planners <_source/tesseract_motion_planners_doc.rst>
   tesseract_process_managers <_source/tesseract_process_managers_doc.rst>
   tesseract_time_parameterization <_source/tesseract_time_parameterization_doc.rst>

FAQ
---
.. toctree::
   :maxdepth: 2

   Questions?<_source/FAQ.rst>

/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef mitkNewAddTool2D_h
#define mitkNewAddTool2D_h

#include "mitkClosedContourTool.h"
#include "mitkContourTool.h"

namespace mitk
{
  /**
    \brief A 2D segmentation tool based on a LiveWire approach.

    The contour between the last point and the current mouse position is
    computed by searching the shortest path according to specific features of
    the image. The contour thus tends to snap to the boundary of objects.

    The tool always assumes that unconfirmed contours are always defined for the
    current time point. So the time step in which the contours will be stored as
    segmentations will be determined when the contours got confirmed. Then they
    will be transfered to the slices of the currently selected time step.
    Changing the time point/time step while tool is active will updated the working
    slice the live wire filter. So the behavior of the active live wire contour is
    always WYSIWYG (What you see is what you get).

    \sa SegTool2D
    \sa ImageLiveWireContourModelFilter

    \ingroup Interaction
    \ingroup ToolManagerEtAl

    \warning Only to be instantiated by mitk::ToolManager.
  */
  class MITKSEGMENTATION_EXPORT NewAddTool2D : public ClosedContourTool
  {
  public:
    mitkClassMacro(NewAddTool2D, SegTool2D);
    itkFactorylessNewMacro(Self);

    us::ModuleResource GetCursorIconResource() const override;
    us::ModuleResource GetIconResource() const override;
    const char *GetName() const override;
    const char **GetXPM() const override;

  protected:
    NewAddTool2D();
    ~NewAddTool2D() override;
  
  };
}

#endif

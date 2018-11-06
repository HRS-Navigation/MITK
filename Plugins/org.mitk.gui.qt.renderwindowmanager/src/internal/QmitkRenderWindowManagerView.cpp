/*===================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center,
Division of Medical and Biological Informatics.
All rights reserved.

This software is distributed WITHOUT ANY WARRANTY; without
even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE.

See LICENSE.txt or http://www.mitk.org for details.

===================================================================*/

// render window manager plugin
#include "QmitkRenderWindowManagerView.h"

// mitk core
#include <mitkBaseRenderer.h>
#include <mitkDataNode.h>

const std::string QmitkRenderWindowManagerView::VIEW_ID = "org.mitk.views.renderwindowmanager";

void QmitkRenderWindowManagerView::SetFocus()
{
  // nothing here
}

void QmitkRenderWindowManagerView::CreateQtPartControl(QWidget* parent)
{
  // create GUI widgets
  m_Controls.setupUi(parent);
  // add custom render window manager UI widget to the 'renderWindowManagerTab'
  m_RenderWindowInspector = new QmitkDataStorageRenderWindowInspector(parent);
  m_RenderWindowInspector->SetDataStorage(GetDataStorage());
  m_RenderWindowInspector->setObjectName(QStringLiteral("m_RenderWindowManipulatorWidget"));
  m_Controls.verticalLayout->addWidget(m_RenderWindowInspector);

  SetControlledRenderer();

  for (const auto& renderer : m_ControlledRenderer)
  {
    m_Controls.comboBoxRenderWindowSelection->addItem(renderer->GetName());
  }

  OnRenderWindowSelectionChanged(m_Controls.comboBoxRenderWindowSelection->itemText(0));

  connect(m_Controls.comboBoxRenderWindowSelection, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(OnRenderWindowSelectionChanged(const QString&)));
}

void QmitkRenderWindowManagerView::SetControlledRenderer()
{
  const mitk::RenderingManager::RenderWindowVector allRegisteredRenderWindows = mitk::RenderingManager::GetInstance()->GetAllRegisteredRenderWindows();
  mitk::BaseRenderer* baseRenderer = nullptr;
  for (const auto &renderWindow : allRegisteredRenderWindows)
  {
    baseRenderer = mitk::BaseRenderer::GetInstance(renderWindow);
    if (nullptr != baseRenderer)
    {
      m_ControlledRenderer.push_back(baseRenderer);
    }
  }

  m_RenderWindowInspector->SetControlledRenderer(m_ControlledRenderer);
}

void QmitkRenderWindowManagerView::OnRenderWindowSelectionChanged(const QString &renderWindowId)
{
  m_RenderWindowInspector->SetActiveRenderWindow(renderWindowId);
}

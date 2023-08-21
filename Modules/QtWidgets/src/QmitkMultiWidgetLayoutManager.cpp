/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#include "QmitkMultiWidgetLayoutManager.h"

#include <mitkLogMacros.h>
#include <QmitkAbstractMultiWidget.h>
#include <QmitkRenderWindow.h>
#include <QmitkRenderWindowWidget.h>

// qt
#include <QHBoxLayout>
#include <qsplitter.h>

QmitkMultiWidgetLayoutManager::QmitkMultiWidgetLayoutManager(QmitkAbstractMultiWidget* multiwidget)
  : QObject(multiwidget)
  , m_MultiWidget(multiwidget)
  , m_CurrentRenderWindowWidget(nullptr)
{
  // nothing here
}

void QmitkMultiWidgetLayoutManager::SetLayoutDesign(LayoutDesign layoutDesign)
{
  if (nullptr == m_MultiWidget)
  {
    return;
  }

  // retrieve the render window name from the sending render window
  auto renderWindow = dynamic_cast<QmitkRenderWindow*>(QObject::sender());
  m_CurrentRenderWindowWidget = m_MultiWidget->GetRenderWindowWidget(renderWindow).get();

  switch (layoutDesign)
  {
  case LayoutDesign::DEFAULT:
  {
    SetDefaultLayout();
    break;
  }
  case LayoutDesign::ALL_2D_TOP_3D_BOTTOM:
  {
    SetAll2DTop3DBottomLayout();
    break;
  }
  case LayoutDesign::ALL_2D_LEFT_3D_RIGHT:
  {
    SetAll2DLeft3DRightLayout();
    break;
  }
  case LayoutDesign::ONE_BIG:
  {
    SetOneBigLayout();
    break;
  }
  case LayoutDesign::ONLY_2D_HORIZONTAL:
  {
    SetOnly2DHorizontalLayout();
    break;
  }
  case LayoutDesign::ONLY_2D_VERTICAL:
  {
    SetOnly2DVerticalLayout();
    break;
  }
  case LayoutDesign::ONE_TOP_3D_BOTTOM:
  {
    SetOneTop3DBottomLayout();
    break;
  }
  case LayoutDesign::ONE_LEFT_3D_RIGHT:
  {
    SetOneLeft3DRightLayout();
    break;
  }
  case LayoutDesign::ALL_HORIZONTAL:
  {
    SetAllHorizontalLayout();
    break;
  }
  case LayoutDesign::ALL_VERTICAL:
  {
    SetAllVerticalLayout();
    break;
  }
  case LayoutDesign::REMOVE_ONE:
  {
    RemoveOneLayout();
    break;
  }
  case LayoutDesign::NONE:
  {
    break;
  }

  // HRS_NAVIGATION_MODIFICATION starts
  case LayoutDesign::AXIAL_LEFT_SAGITTAL_RIGHT:
  {
    SetAxialLeftSagittalRightLayout();
    break;
  }
  case LayoutDesign::AXIAL_SAGITTAL_LEFT_3D_RIGHT:
  {
    SetAxialSagittalLeft3DRightLayout();
    break;
  }
  case LayoutDesign::AXIAL_3D_LEFT_SAGITTAL_RIGHT:
  {
    SetAxial3DLeftSagittalRightLayout();
    break;
  }
    // HRS_NAVIGATION_MODIFICATION ends
  };
}

void QmitkMultiWidgetLayoutManager::SetCurrentRenderWindowWidget(QmitkRenderWindowWidget* renderWindowWidget)
{
  m_CurrentRenderWindowWidget = renderWindowWidget;
}

void QmitkMultiWidgetLayoutManager::SetDefaultLayout()
{
  MITK_INFO << "Set default layout" << std::endl;

  m_MultiWidget->ActivateMenuWidget(false);

  delete m_MultiWidget->layout();

  auto hBoxLayout = new QHBoxLayout(m_MultiWidget);
  hBoxLayout->setContentsMargins(0, 0, 0, 0);
  m_MultiWidget->setLayout(hBoxLayout);
  hBoxLayout->setMargin(0);

  auto mainSplit = new QSplitter(Qt::Vertical, m_MultiWidget);
  hBoxLayout->addWidget(mainSplit);

  QList<int> splitterSizeRow;
  for (int row = 0; row < m_MultiWidget->GetRowCount(); ++row)
  {
    splitterSizeRow.push_back(1000);

    QList<int> splitterSizeColumn;
    auto splitter = new QSplitter(mainSplit);
    for (int column = 0; column < m_MultiWidget->GetColumnCount(); ++column)
    {
      splitterSizeColumn.push_back(1000);
      auto renderWindowWidget = m_MultiWidget->GetRenderWindowWidget(row, column);
      splitter->addWidget(renderWindowWidget.get());
      renderWindowWidget->show();
    }
    splitter->setSizes(splitterSizeColumn);
  }

  mainSplit->setSizes(splitterSizeRow);

  m_MultiWidget->ActivateMenuWidget(true);

  auto allRenderWindows = m_MultiWidget->GetRenderWindows();
  for (auto& renderWindow : allRenderWindows)
  {
    renderWindow->UpdateLayoutDesignList(LayoutDesign::DEFAULT);
  }
}

void QmitkMultiWidgetLayoutManager::SetAll2DTop3DBottomLayout()
{
  MITK_INFO << "Set all 2D top and 3D bottom layout" << std::endl;

  m_MultiWidget->ActivateMenuWidget(false);

  delete m_MultiWidget->layout();

  auto hBoxLayout = new QHBoxLayout(m_MultiWidget);
  hBoxLayout->setContentsMargins(0, 0, 0, 0);
  m_MultiWidget->setLayout(hBoxLayout);
  hBoxLayout->setMargin(0);

  auto mainSplit = new QSplitter(Qt::Vertical, m_MultiWidget);
  hBoxLayout->addWidget(mainSplit);

  auto subSplit2D = new QSplitter(mainSplit);
  QList<int> splitterSize;
  auto all2DRenderWindowWidgets = m_MultiWidget->Get2DRenderWindowWidgets();
  for (const auto& renderWindowWidget : all2DRenderWindowWidgets)
  {
    subSplit2D->addWidget(renderWindowWidget.second.get());
    renderWindowWidget.second->show();
    splitterSize.push_back(1000);
  }
  subSplit2D->setSizes(splitterSize);

  auto subSplit3D = new QSplitter(mainSplit);
  splitterSize.clear();
  auto all3DRenderWindowWidgets = m_MultiWidget->Get3DRenderWindowWidgets();
  for (const auto& renderWindowWidget : all3DRenderWindowWidgets)
  {
    subSplit3D->addWidget(renderWindowWidget.second.get());
    renderWindowWidget.second->show();
    splitterSize.push_back(1000);
  }
  subSplit3D->setSizes(splitterSize);

  // set size for main splitter
  splitterSize.clear();
  splitterSize.push_back(600);
  splitterSize.push_back(1000);
  mainSplit->setSizes(splitterSize);

  m_MultiWidget->ActivateMenuWidget(true);

  auto allRenderWindows = m_MultiWidget->GetRenderWindows();
  for (auto& renderWindow : allRenderWindows)
  {
    renderWindow->UpdateLayoutDesignList(LayoutDesign::ALL_2D_TOP_3D_BOTTOM);
  }
}

void QmitkMultiWidgetLayoutManager::SetAll2DLeft3DRightLayout()
{
  MITK_INFO << "Set all 2D left and 3D right layout" << std::endl;

  m_MultiWidget->ActivateMenuWidget(false);

  delete m_MultiWidget->layout();

  auto hBoxLayout = new QHBoxLayout(m_MultiWidget);
  hBoxLayout->setContentsMargins(0, 0, 0, 0);
  m_MultiWidget->setLayout(hBoxLayout);
  hBoxLayout->setMargin(0);

  auto mainSplit = new QSplitter(m_MultiWidget);
  hBoxLayout->addWidget(mainSplit);

  auto subSplit2D = new QSplitter(Qt::Vertical, mainSplit);
  QList<int> splitterSize;
  auto all2DRenderWindowWidgets = m_MultiWidget->Get2DRenderWindowWidgets();
  for (const auto& renderWindowWidget : all2DRenderWindowWidgets)
  {
    subSplit2D->addWidget(renderWindowWidget.second.get());
    renderWindowWidget.second->show();
    splitterSize.push_back(1000);
  }
  subSplit2D->setSizes(splitterSize);

  auto subSplit3D = new QSplitter(mainSplit);
  splitterSize.clear();
  auto all3DRenderWindowWidgets = m_MultiWidget->Get3DRenderWindowWidgets();
  for (const auto& renderWindowWidget : all3DRenderWindowWidgets)
  {
    subSplit3D->addWidget(renderWindowWidget.second.get());
    renderWindowWidget.second->show();
    splitterSize.push_back(1000);
  }
  subSplit3D->setSizes(splitterSize);

  // set size for main splitter
  splitterSize.clear();
  splitterSize.push_back(600);
  splitterSize.push_back(1000);
  mainSplit->setSizes(splitterSize);

  m_MultiWidget->ActivateMenuWidget(true);

  auto allRenderWindows = m_MultiWidget->GetRenderWindows();
  for (auto& renderWindow : allRenderWindows)
  {
    renderWindow->UpdateLayoutDesignList(LayoutDesign::ALL_2D_LEFT_3D_RIGHT);
  }
}

void QmitkMultiWidgetLayoutManager::SetOneBigLayout()
{
  MITK_INFO << "Set single 2D layout" << std::endl;

  m_MultiWidget->ActivateMenuWidget(false);

  delete m_MultiWidget->layout();

  auto hBoxLayout = new QHBoxLayout(m_MultiWidget);
  hBoxLayout->setContentsMargins(0, 0, 0, 0);
  m_MultiWidget->setLayout(hBoxLayout);
  hBoxLayout->setMargin(0);

  auto mainSplit = new QSplitter(m_MultiWidget);
  hBoxLayout->addWidget(mainSplit);
  mainSplit->addWidget(m_CurrentRenderWindowWidget);
  m_CurrentRenderWindowWidget->show();

  m_MultiWidget->ActivateMenuWidget(true);

  auto allRenderWindows = m_MultiWidget->GetRenderWindows();
  for (auto& renderWindow : allRenderWindows)
  {
    renderWindow->UpdateLayoutDesignList(LayoutDesign::ONE_BIG);
  }
}

void QmitkMultiWidgetLayoutManager::SetOnly2DHorizontalLayout()
{
  MITK_INFO << "Set only 2D layout" << std::endl;

  m_MultiWidget->ActivateMenuWidget(false);

  delete m_MultiWidget->layout();

  auto hBoxLayout = new QHBoxLayout(m_MultiWidget);
  hBoxLayout->setContentsMargins(0, 0, 0, 0);
  m_MultiWidget->setLayout(hBoxLayout);
  hBoxLayout->setMargin(0);

  auto mainSplit = new QSplitter(m_MultiWidget);
  hBoxLayout->addWidget(mainSplit);

  QList<int> splitterSize;
  auto all2DRenderWindowWidgets = m_MultiWidget->Get2DRenderWindowWidgets();
  for (const auto& renderWindowWidget : all2DRenderWindowWidgets)
  {
    mainSplit->addWidget(renderWindowWidget.second.get());
    renderWindowWidget.second->show();
    splitterSize.push_back(1000);
  }
  mainSplit->setSizes(splitterSize);

  m_MultiWidget->ActivateMenuWidget(true);

  // HRS_NAVIGATION_MODIFICATION starts
  // as here we are just showing the 2D windows hence lets hide the 3D windows.
  auto all3DRenderWindowWidgets = m_MultiWidget->Get3DRenderWindowWidgets();
  for (const auto &renderWindowWidget : all3DRenderWindowWidgets)
  {
    renderWindowWidget.second->hide();
  }
  // HRS_NAVIGATION_MODIFICATION ends

  auto allRenderWindows = m_MultiWidget->GetRenderWindows();
  for (auto& renderWindow : allRenderWindows)
  {
    renderWindow->UpdateLayoutDesignList(LayoutDesign::ONLY_2D_HORIZONTAL);
  }
}

void QmitkMultiWidgetLayoutManager::SetOnly2DVerticalLayout()
{
  MITK_INFO << "Set only 2D layout" << std::endl;

  m_MultiWidget->ActivateMenuWidget(false);

  delete m_MultiWidget->layout();

  auto hBoxLayout = new QHBoxLayout(m_MultiWidget);
  hBoxLayout->setContentsMargins(0, 0, 0, 0);
  m_MultiWidget->setLayout(hBoxLayout);
  hBoxLayout->setMargin(0);

  auto mainSplit = new QSplitter(Qt::Vertical, m_MultiWidget);
  hBoxLayout->addWidget(mainSplit);

  QList<int> splitterSize;
  auto all2DRenderWindowWidgets = m_MultiWidget->Get2DRenderWindowWidgets();
  for (const auto& renderWindowWidget : all2DRenderWindowWidgets)
  {
    mainSplit->addWidget(renderWindowWidget.second.get());
    renderWindowWidget.second->show();
    splitterSize.push_back(1000);
  }
  mainSplit->setSizes(splitterSize);

  m_MultiWidget->ActivateMenuWidget(true);

  // HRS_NAVIGATION_MODIFICATION starts
  // as here we are just showing the 2D windows hence lets hide the 3D windows.
  auto all3DRenderWindowWidgets = m_MultiWidget->Get3DRenderWindowWidgets();
  for (const auto &renderWindowWidget : all3DRenderWindowWidgets)
  {
    renderWindowWidget.second->hide();
  }
  // HRS_NAVIGATION_MODIFICATION ends

  auto allRenderWindows = m_MultiWidget->GetRenderWindows();
  for (auto& renderWindow : allRenderWindows)
  {
    renderWindow->UpdateLayoutDesignList(LayoutDesign::ONLY_2D_VERTICAL);
  }
}

void QmitkMultiWidgetLayoutManager::SetOneTop3DBottomLayout()
{
  MITK_INFO << "Set one top and all 3D bottom layout" << std::endl;

  m_MultiWidget->ActivateMenuWidget(false);

  delete m_MultiWidget->layout();

  auto hBoxLayout = new QHBoxLayout(m_MultiWidget);
  hBoxLayout->setContentsMargins(0, 0, 0, 0);
  m_MultiWidget->setLayout(hBoxLayout);
  hBoxLayout->setMargin(0);

  auto mainSplit = new QSplitter(Qt::Vertical, m_MultiWidget);
  hBoxLayout->addWidget(mainSplit);

  mainSplit->addWidget(m_CurrentRenderWindowWidget);
  m_CurrentRenderWindowWidget->show();

  auto subSplit3D = new QSplitter(mainSplit);
  QList<int> splitterSize;
  auto all3DRenderWindowWidgets = m_MultiWidget->Get3DRenderWindowWidgets();
  for (const auto& renderWindowWidget : all3DRenderWindowWidgets)
  {
    subSplit3D->addWidget(renderWindowWidget.second.get());
    renderWindowWidget.second->show();
    splitterSize.push_back(1000);
  }
  subSplit3D->setSizes(splitterSize);

  // set size for main splitter
  splitterSize.clear();
  splitterSize.push_back(1000);
  splitterSize.push_back(1000);
  mainSplit->setSizes(splitterSize);

  m_MultiWidget->ActivateMenuWidget(true);

  auto allRenderWindows = m_MultiWidget->GetRenderWindows();
  for (auto& renderWindow : allRenderWindows)
  {
    renderWindow->UpdateLayoutDesignList(LayoutDesign::ONE_TOP_3D_BOTTOM);
  }
}

void QmitkMultiWidgetLayoutManager::SetOneLeft3DRightLayout()
{
  MITK_INFO << "Set one left and all 3D right layout" << std::endl;

  m_MultiWidget->ActivateMenuWidget(false);

  delete m_MultiWidget->layout();

  auto hBoxLayout = new QHBoxLayout(m_MultiWidget);
  hBoxLayout->setContentsMargins(0, 0, 0, 0);
  m_MultiWidget->setLayout(hBoxLayout);
  hBoxLayout->setMargin(0);

  auto mainSplit = new QSplitter(m_MultiWidget);
  hBoxLayout->addWidget(mainSplit);

  mainSplit->addWidget(m_CurrentRenderWindowWidget);
  m_CurrentRenderWindowWidget->show();

  auto subSplit3D = new QSplitter(Qt::Vertical, mainSplit);
  QList<int> splitterSize;
  auto all3DRenderWindowWidgets = m_MultiWidget->Get3DRenderWindowWidgets();
  for (const auto& renderWindowWidget : all3DRenderWindowWidgets)
  {
    subSplit3D->addWidget(renderWindowWidget.second.get());
    renderWindowWidget.second->show();
    splitterSize.push_back(1000);
  }
  subSplit3D->setSizes(splitterSize);

  // set size for main splitter
  splitterSize.clear();
  splitterSize.push_back(1000);
  splitterSize.push_back(1000);
  mainSplit->setSizes(splitterSize);

  m_MultiWidget->ActivateMenuWidget(true);

  auto allRenderWindows = m_MultiWidget->GetRenderWindows();
  for (auto& renderWindow : allRenderWindows)
  {
    renderWindow->UpdateLayoutDesignList(LayoutDesign::ONE_LEFT_3D_RIGHT);
  }
}

void QmitkMultiWidgetLayoutManager::SetAllHorizontalLayout()
{
  MITK_INFO << "Set default layout" << std::endl;

  m_MultiWidget->ActivateMenuWidget(false);

  delete m_MultiWidget->layout();

  auto hBoxLayout = new QHBoxLayout(m_MultiWidget);
  hBoxLayout->setContentsMargins(0, 0, 0, 0);
  m_MultiWidget->setLayout(hBoxLayout);
  hBoxLayout->setMargin(0);

  auto mainSplit = new QSplitter(m_MultiWidget);
  hBoxLayout->addWidget(mainSplit);

  QList<int> splitterSize;
  auto allRenderWindowWidgets = m_MultiWidget->GetRenderWindowWidgets();
  for (const auto& renderWindowWidget : allRenderWindowWidgets)
  {
    if (nullptr != renderWindowWidget.second)
    {
      mainSplit->addWidget(renderWindowWidget.second.get());
      renderWindowWidget.second->show();
      splitterSize.push_back(1000);
    }
  }

  // set size for main splitter
  mainSplit->setSizes(splitterSize);

  m_MultiWidget->ActivateMenuWidget(true);

  auto allRenderWindows = m_MultiWidget->GetRenderWindows();
  for (auto& renderWindow : allRenderWindows)
  {
    renderWindow->UpdateLayoutDesignList(LayoutDesign::ALL_HORIZONTAL);
  }
}

void QmitkMultiWidgetLayoutManager::SetAllVerticalLayout()
{
  MITK_INFO << "Set all vertical" << std::endl;

  m_MultiWidget->ActivateMenuWidget(false);

  delete m_MultiWidget->layout();

  auto hBoxLayout = new QHBoxLayout(m_MultiWidget);
  hBoxLayout->setContentsMargins(0, 0, 0, 0);
  m_MultiWidget->setLayout(hBoxLayout);
  hBoxLayout->setMargin(0);

  auto mainSplit = new QSplitter(Qt::Vertical, m_MultiWidget);
  hBoxLayout->addWidget(mainSplit);

  QList<int> splitterSize;
  auto allRenderWindowWidgets = m_MultiWidget->GetRenderWindowWidgets();
  for (const auto& renderWindowWidget : allRenderWindowWidgets)
  {
    mainSplit->addWidget(renderWindowWidget.second.get());
    renderWindowWidget.second->show();
    splitterSize.push_back(1000);
  }

  // set size for splitter
  mainSplit->setSizes(splitterSize);

  m_MultiWidget->ActivateMenuWidget(true);

  auto allRenderWindows = m_MultiWidget->GetRenderWindows();
  for (auto& renderWindow : allRenderWindows)
  {
    renderWindow->UpdateLayoutDesignList(LayoutDesign::ALL_VERTICAL);
  }
}

void QmitkMultiWidgetLayoutManager::RemoveOneLayout()
{
  MITK_INFO << "Remove single render window" << std::endl;

  m_MultiWidget->ActivateMenuWidget(false);

  m_CurrentRenderWindowWidget->hide();

  m_MultiWidget->ActivateMenuWidget(true);

  auto allRenderWindows = m_MultiWidget->GetRenderWindows();
  for (auto& renderWindow : allRenderWindows)
  {
    renderWindow->UpdateLayoutDesignList(LayoutDesign::NONE);
  }
}




// HRS_NAVIGATION_MODIFICATION starts

//------------------------------------------------------------------------------------
//
void QmitkMultiWidgetLayoutManager::SetAxialLeftSagittalRightLayout()
{
  MITK_INFO << "Set Axial n Sagittal layout" << std::endl;

  m_MultiWidget->ActivateMenuWidget(false);

  delete m_MultiWidget->layout();

  auto hBoxLayout = new QHBoxLayout(m_MultiWidget);
  hBoxLayout->setContentsMargins(0, 0, 0, 0);
  m_MultiWidget->setLayout(hBoxLayout);
  hBoxLayout->setMargin(0);

  auto mainSplit = new QSplitter(m_MultiWidget);
  hBoxLayout->addWidget(mainSplit);

  QmitkRenderWindowWidget *clpAxialRenderWindowWidget = nullptr;
  QmitkRenderWindowWidget *clpSagittalRenderWindowWidget = nullptr;
  auto all2DRenderWindowWidgets = m_MultiWidget->Get2DRenderWindowWidgets();
  for (const auto &renderWindowWidget : all2DRenderWindowWidgets)
  {
    if (renderWindowWidget.second)
    {
      if (mitk::BaseRenderer::ViewDirection::AXIAL == renderWindowWidget.second.get()->GetRenderWindow()->GetLayoutIndex() )
        clpAxialRenderWindowWidget = renderWindowWidget.second.get();

      if (mitk::BaseRenderer::ViewDirection::SAGITTAL == renderWindowWidget.second.get()->GetRenderWindow()->GetLayoutIndex() )
        clpSagittalRenderWindowWidget = renderWindowWidget.second.get();
    }
  }

   QList<int> splitterSize;
  if (clpAxialRenderWindowWidget)
  {
    mainSplit->addWidget(clpAxialRenderWindowWidget);
    clpAxialRenderWindowWidget->show();
    splitterSize.push_back(1000);
  }
  if (clpSagittalRenderWindowWidget)
  {
    mainSplit->addWidget(clpSagittalRenderWindowWidget);
    clpSagittalRenderWindowWidget->show();
    splitterSize.push_back(1000);
  }

  // set size for main splitter
  mainSplit->setSizes(splitterSize);


  m_MultiWidget->ActivateMenuWidget(true);

  auto allRenderWindows = m_MultiWidget->GetRenderWindows();
  for (auto &renderWindow : allRenderWindows)
  {
    renderWindow->UpdateLayoutDesignList(LayoutDesign::AXIAL_LEFT_SAGITTAL_RIGHT);
  }
}

//------------------------------------------------------------------------------------
//
void QmitkMultiWidgetLayoutManager::SetAxialSagittalLeft3DRightLayout()
{
  MITK_INFO << "Set Axial Sagittal Left 3D Right layout" << std::endl;

  m_MultiWidget->ActivateMenuWidget(false);

  delete m_MultiWidget->layout();

  auto hBoxLayout = new QHBoxLayout(m_MultiWidget);
  hBoxLayout->setContentsMargins(0, 0, 0, 0);
  m_MultiWidget->setLayout(hBoxLayout);
  hBoxLayout->setMargin(0);

  auto mainSplit = new QSplitter(m_MultiWidget);
  hBoxLayout->addWidget(mainSplit);

  QmitkRenderWindowWidget *clpAxialRenderWindowWidget = nullptr;
  QmitkRenderWindowWidget *clpSagittalRenderWindowWidget = nullptr;
  QmitkRenderWindowWidget *clp3DRenderWindowWidget = nullptr;
  {
    auto all2DRenderWindowWidgets = m_MultiWidget->Get2DRenderWindowWidgets();
    for (const auto &renderWindowWidget : all2DRenderWindowWidgets)
    {
      if (renderWindowWidget.second)
      {
        if (mitk::BaseRenderer::ViewDirection::AXIAL ==
            renderWindowWidget.second.get()->GetRenderWindow()->GetLayoutIndex())
          clpAxialRenderWindowWidget = renderWindowWidget.second.get();

        if (mitk::BaseRenderer::ViewDirection::SAGITTAL ==
            renderWindowWidget.second.get()->GetRenderWindow()->GetLayoutIndex())
          clpSagittalRenderWindowWidget = renderWindowWidget.second.get();
      }
    }

    auto all3DRenderWindowWidgets = m_MultiWidget->Get3DRenderWindowWidgets();
    for (const auto &renderWindowWidget : all3DRenderWindowWidgets)
    {
      if (renderWindowWidget.second)
      {
        clp3DRenderWindowWidget = renderWindowWidget.second.get();
        break;
      }
    }
  }

  auto subSplit2D = new QSplitter(Qt::Vertical, mainSplit);
  QList<int> splitterSize;
  if (clpAxialRenderWindowWidget)
  {
    subSplit2D->addWidget(clpAxialRenderWindowWidget);
    clpAxialRenderWindowWidget->show();
    splitterSize.push_back(1000);
  }
  if (clpSagittalRenderWindowWidget)
  {
    subSplit2D->addWidget(clpSagittalRenderWindowWidget);
    clpSagittalRenderWindowWidget->show();
    splitterSize.push_back(1000);
  }
  subSplit2D->setSizes(splitterSize);

  auto subSplit3D = new QSplitter(mainSplit);
  splitterSize.clear();
  if (clp3DRenderWindowWidget)
  {
    subSplit3D->addWidget(clp3DRenderWindowWidget);
    clp3DRenderWindowWidget->show();
    splitterSize.push_back(1000);
  }
  subSplit3D->setSizes(splitterSize);

  // set size for main splitter
  splitterSize.clear();
  splitterSize.push_back(600);
  splitterSize.push_back(1000);
  mainSplit->setSizes(splitterSize);

  m_MultiWidget->ActivateMenuWidget(true);

  auto allRenderWindows = m_MultiWidget->GetRenderWindows();
  for (auto &renderWindow : allRenderWindows)
  {
    renderWindow->UpdateLayoutDesignList(LayoutDesign::AXIAL_SAGITTAL_LEFT_3D_RIGHT);
  }
}

//------------------------------------------------------------------------------------
//
void QmitkMultiWidgetLayoutManager::SetAxial3DLeftSagittalRightLayout()
{
  MITK_INFO << "Set Axial 3D Left Sagittal Right layout" << std::endl;

  m_MultiWidget->ActivateMenuWidget(false);

  delete m_MultiWidget->layout();

  auto hBoxLayout = new QHBoxLayout(m_MultiWidget);
  hBoxLayout->setContentsMargins(0, 0, 0, 0);
  m_MultiWidget->setLayout(hBoxLayout);
  hBoxLayout->setMargin(0);

  auto mainSplit = new QSplitter(m_MultiWidget);
  hBoxLayout->addWidget(mainSplit);

  QmitkRenderWindowWidget *clpAxialRenderWindowWidget = nullptr;
  QmitkRenderWindowWidget *clpSagittalRenderWindowWidget = nullptr;
  QmitkRenderWindowWidget *clp3DRenderWindowWidget = nullptr;
  {
    auto all2DRenderWindowWidgets = m_MultiWidget->Get2DRenderWindowWidgets();
    for (const auto &renderWindowWidget : all2DRenderWindowWidgets)
    {
      if (renderWindowWidget.second)
      {
        if (mitk::BaseRenderer::ViewDirection::AXIAL ==
            renderWindowWidget.second.get()->GetRenderWindow()->GetLayoutIndex())
          clpAxialRenderWindowWidget = renderWindowWidget.second.get();

        if (mitk::BaseRenderer::ViewDirection::SAGITTAL ==
            renderWindowWidget.second.get()->GetRenderWindow()->GetLayoutIndex())
          clpSagittalRenderWindowWidget = renderWindowWidget.second.get();
      }
    }

    auto all3DRenderWindowWidgets = m_MultiWidget->Get3DRenderWindowWidgets();
    for (const auto &renderWindowWidget : all3DRenderWindowWidgets)
    {
      if (renderWindowWidget.second)
      {
        clp3DRenderWindowWidget = renderWindowWidget.second.get();
        break;
      }
    }
  }

  auto subSplitLeft = new QSplitter(Qt::Vertical, mainSplit);
  QList<int> splitterSize;
  if (clpAxialRenderWindowWidget)
  {
    subSplitLeft->addWidget(clpAxialRenderWindowWidget);
    clpAxialRenderWindowWidget->show();
    splitterSize.push_back(1000);
  }
  if (clp3DRenderWindowWidget)
  {
    subSplitLeft->addWidget(clp3DRenderWindowWidget);
    clp3DRenderWindowWidget->show();
    splitterSize.push_back(1000);
  }
  subSplitLeft->setSizes(splitterSize);


  auto subSplitRight = new QSplitter(mainSplit);
  splitterSize.clear();

  if (clpSagittalRenderWindowWidget)
  {
    subSplitRight->addWidget(clpSagittalRenderWindowWidget);
    clpSagittalRenderWindowWidget->show();
    splitterSize.push_back(1000);
  }
  subSplitRight->setSizes(splitterSize);


  // set size for main splitter
  splitterSize.clear();
  splitterSize.push_back(600);
  splitterSize.push_back(1000);
  mainSplit->setSizes(splitterSize);

  m_MultiWidget->ActivateMenuWidget(true);

  auto allRenderWindows = m_MultiWidget->GetRenderWindows();
  for (auto &renderWindow : allRenderWindows)
  {
    renderWindow->UpdateLayoutDesignList(LayoutDesign::AXIAL_3D_LEFT_SAGITTAL_RIGHT);
  }
}
// HRS_NAVIGATION_MODIFICATION ends

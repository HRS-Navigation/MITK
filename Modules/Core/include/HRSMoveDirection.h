// HRS_NAVIGATION_MODIFICATION starts

#ifndef __HRSMoveDirection_h_
#define __HRSMoveDirection_h_

#include "mitkBaseRenderer.h"

//------------------------------------------------------------------------------------
// Added by AmitRungta on 03-04-2023
//
class MoveDirectionHelper
{
public:
  MoveDirectionHelper()
  {
    m_clAxialMoveDirection = MoveDirection(true, false, false);
    m_clSagittalMoveDirection = MoveDirection(true, false, true);
    m_clCoronalMoveDirection = MoveDirection(true, false, false);
  };
  ~MoveDirectionHelper(){};

public:
  struct MoveDirection
  {
    bool m_bMoveFootToHead = true;           // This will show head at top os screen
    bool m_bMoveLeftToRight = false;         // This will show Right portion of Body to Left Side
    bool m_bMoveAnteriorToPosterior = false; // THis will show Nose at Bottom of the screen.

    MoveDirection(bool baMoveFootToHead = true, bool baMoveLeftToRight = false, bool baMoveAnteriorToPosterior = false)
    {
      m_bMoveFootToHead = baMoveFootToHead;
      m_bMoveLeftToRight = baMoveLeftToRight;
      m_bMoveAnteriorToPosterior = baMoveAnteriorToPosterior;
    }
  };

public:
  MoveDirection *fnGetMoveMoveDirectionData(mitk::BaseRenderer::ViewDirection enmaViewDirection)
  {
    switch (enmaViewDirection)
    {
      case mitk::BaseRenderer::ViewDirection::AXIAL:
        return &m_clAxialMoveDirection;

      case mitk::BaseRenderer::ViewDirection::SAGITTAL:
        return &m_clSagittalMoveDirection;

      case mitk::BaseRenderer::ViewDirection::CORONAL:
        return &m_clCoronalMoveDirection;
    }

    return &m_clAxialMoveDirection;
  }

  void fnSetMoveDirections(mitk::BaseRenderer::ViewDirection enmaViewDirection,
                           bool baMoveFootToHead,
                           bool baMoveLeftToRight,
                           bool baMoveAnteriorToPosterior)
  {
    MoveDirection *clpMoveDirection = fnGetMoveMoveDirectionData(enmaViewDirection);
    clpMoveDirection->m_bMoveFootToHead = baMoveFootToHead;
    clpMoveDirection->m_bMoveLeftToRight = baMoveLeftToRight;
    clpMoveDirection->m_bMoveAnteriorToPosterior = baMoveAnteriorToPosterior;
  }


  bool fnGetMoveFootToHead(mitk::BaseRenderer::ViewDirection enmaViewDirection)
  {
    return fnGetMoveMoveDirectionData(enmaViewDirection)->m_bMoveFootToHead;
  };
  bool fnGetMoveLeftToRight(mitk::BaseRenderer::ViewDirection enmaViewDirection)
  {
    return fnGetMoveMoveDirectionData(enmaViewDirection)->m_bMoveLeftToRight;
  }
  bool fnGetMoveAnteriorToPosterior(mitk::BaseRenderer::ViewDirection enmaViewDirection)
  {
    return fnGetMoveMoveDirectionData(enmaViewDirection)->m_bMoveAnteriorToPosterior;
  }


protected:
  MoveDirection m_clAxialMoveDirection, m_clSagittalMoveDirection, m_clCoronalMoveDirection;
};

#endif // #define __HRSMoveDirection_h_
// HRS_NAVIGATION_MODIFICATION ends

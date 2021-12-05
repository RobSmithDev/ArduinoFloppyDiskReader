#include "TColorText.h"

void TColorText::setTransparent(bool ATransparent)
{
    MTransparent = ATransparent;
    Invalidate();
}

void TColorText::SetBackgroundColor(COLORREF AColor)
{
    MBackgroundColor = AColor;
    MTransparent = false;
    Invalidate();
}

void TColorText::SetTextColor(COLORREF AColor)
{
    MTextColor = AColor;
    Invalidate();
}

BEGIN_MESSAGE_MAP(TColorText, CStatic)
    ON_WM_CTLCOLOR_REFLECT()
END_MESSAGE_MAP()

HBRUSH TColorText::CtlColor(CDC* pDC, UINT nCtlColor)
{
    pDC->SetTextColor(MTextColor);
    pDC->SetBkMode(TRANSPARENT);  // we do not want to draw background when drawing text. 
                                    // background color comes from drawing the control background.
    if (MTransparent)
        return nullptr;  // return nullptr to indicate that the parent object 
                         // should supply the brush. it has the appropriate background color.
    else
        return (HBRUSH)CreateSolidBrush(MBackgroundColor);  // color for the empty area of the control
}


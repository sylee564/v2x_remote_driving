#include "QtWidgetGroup.h"

void QtWidgetGroup::addWidget(QWidget* widget) {
    widgets.push_back(widget);
}

void QtWidgetGroup::enableButtons(const bool& enable) {
    for (auto widget = widgets.begin(); widget != widgets.end(); ++widget) {
        (*widget)->setEnabled(enable);
    }
}

void QtWidgetGroup::switchFocusTo(const QWidget* focusWidget, const QString& backgroundStr) {
    for (auto widget = widgets.begin(); widget != widgets.end(); ++widget) {
        if ( (*widget) == focusWidget ) {
            (*widget)->setStyleSheet(backgroundStr);
        } else {
            (*widget)->setStyleSheet("");
        }
    }
}
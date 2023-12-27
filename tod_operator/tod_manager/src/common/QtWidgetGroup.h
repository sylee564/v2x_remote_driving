#include <QtWidgets>
#include <vector>

class QtWidgetGroup {
public:
    QtWidgetGroup() = default;
    void addWidget(QWidget* widget);
    void enableButtons(const bool& enable);
    void switchFocusTo(const QWidget* focusWidget, const QString& backgroundStr);

private:
    std::vector<QWidget*> widgets;
};

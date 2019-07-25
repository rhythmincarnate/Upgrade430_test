#ifndef UPGRADEFORM_H
#define UPGRADEFORM_H

#include <QFrame>

namespace Ui {
class UpgradeForm;

}

class UpgradeForm : public QFrame
{
    Q_OBJECT

public:
    explicit UpgradeForm(uint8_t id,QWidget *parent = 0);
    ~UpgradeForm();
signals:
    void startUpgrade();
public slots:
    void onStateChanged(uint8_t state);
    void upgrade();
    void upgradePercent(int percent);
private slots:
    void on_upgrade_clicked();

private:
    Ui::UpgradeForm *ui;
    uint8_t m_actuatorId;
};

#endif // UPGRADEFORM_H

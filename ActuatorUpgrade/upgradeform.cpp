#include "upgradeform.h"
#include "ui_upgradeform.h"
#include "upgradeunit.h"

UpgradeForm::UpgradeForm(uint8_t id, QWidget *parent) :
    QFrame(parent),
    ui(new Ui::UpgradeForm),
    m_actuatorId(id)
{
    ui->setupUi(this);
    ui->id->setText(tr("%1").arg(id));


}

UpgradeForm::~UpgradeForm()
{
    delete ui;
}

void UpgradeForm::onStateChanged(uint8_t state)
{
    switch (state) {
    case UpgradeUnit::Upgrade_Failed:
    case UpgradeUnit::Upgrade_Timeout:
        ui->state->setText("Failed.");
        break;
    case UpgradeUnit::Upgrade_Finished:
        ui->state->setText("Finished.");
        break;
    case UpgradeUnit::Upgrade_Sending:
        ui->state->setText("Upgrading.");
        if(ui->upgrade->isEnabled())
        {
            ui->upgrade->setEnabled(false);
        }
        break;
    case UpgradeUnit::Upgrade_Nofile:
        ui->state->setText("No file");
        break;
    default:
        break;
    }
}

void UpgradeForm::upgrade()
{
    emit startUpgrade();
    ui->upgrade->setEnabled(false);
}

void UpgradeForm::upgradePercent(int percent)
{
    ui->percent->setValue(percent);
}

void UpgradeForm::on_upgrade_clicked()
{
    upgrade();
}

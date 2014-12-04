#include "widgets/state_widget.h"

state_widget::state_widget()
{
    status_table.setColumnCount(2);
    QStringList labels;
    labels << "Module" << "Message";
    status_table.setHeaderLabels(labels);
    status_table.setRootIsDecorated(false);
        
    QTreeWidgetItem* item = new QTreeWidgetItem();
    item->setText(0, QString::fromStdString("Prova"));
    item->setText(1, QString::fromStdString("Ciao"));
    status_table.insertTopLevelItem(0,item);
    
    main_layout.addWidget(&status_table,0,0,Qt::AlignCenter);
    
    setLayout(&main_layout);
}

state_widget::~state_widget()
{

}
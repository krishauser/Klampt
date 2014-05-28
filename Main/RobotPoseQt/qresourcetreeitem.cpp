#include "qresourcetreeitem.h"

QResourceTreeItem::QResourceTreeItem(ResourceNodePtr rt)
{
    resource = rt;
    string name=rt->Name();
    setText(NAMECOL,QString::fromStdString(name));
    string type = rt->resource->Type();
    if(!type.empty())
        setText(TYPECOL,QString::fromStdString(rt->resource->Type()));

    if(rt->IsExpandable()) {
      setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable | Qt::ItemIsSelectable | Qt::ItemIsDropEnabled | Qt::ItemIsDragEnabled);
      setChildIndicatorPolicy(QTreeWidgetItem::ShowIndicator);
    }
    else {
      setChildIndicatorPolicy(QTreeWidgetItem::DontShowIndicator);
      setFlags(Qt::ItemIsEnabled | Qt::ItemIsEditable | Qt::ItemIsSelectable | Qt::ItemIsDragEnabled);
    }

    updateDecorator();

    AddChildren(rt->children);
}


void QResourceTreeItem::addChild(ResourceNodePtr r){
    QResourceTreeItem* exp=new QResourceTreeItem(r);
    QTreeWidgetItem::addChild(exp);
}

void QResourceTreeItem::AddChildren(const vector<ResourceNodePtr>& resources){
    for(size_t i=0;i<resources.size();i++){
        addChild(resources[i]);
    }
}

void QResourceTreeItem::updateDecorator()
{
  setText(DECORATORCOL,QString(resource->Decorator()));
}

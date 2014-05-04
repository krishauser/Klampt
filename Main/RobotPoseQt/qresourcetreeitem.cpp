#include "qresourcetreeitem.h"

QResourceTreeItem::QResourceTreeItem()
{
}

//QResourceTreeItem::QResourceTreeItem(QString name,QString type){
//    setText(NAMECOL,name);
//    setText(TYPECOL,type);
//}

QResourceTreeItem::QResourceTreeItem(ResourceNode rt){
    resource=rt;
    name=rt->Name();
    setText(NAMECOL,QString::fromStdString(name));
    string type = rt->resource->Type();
    if(!type.empty())
        setText(TYPECOL,QString::fromStdString(rt->resource->Type()));
    dirty = 0;
    show_delete = 1;
    show_expand = 1;
    show_load = 1;
    show_togui = 1;

}

void QResourceTreeItem::ToGUI()
{printf("virtual class");}

void QResourceTreeItem::FromGUI()
{printf("virtual class");}

void QResourceTreeItem::DeleteItem()
{
    printf("delete this?");
    //if(parent.) delete this;
}

void QResourceTreeItem::addChild(ResourceNode r){
    QResourceTreeItem* exp=new QResourceTreeItem(r);
    QTreeWidgetItem::addChild(exp);
}

void QResourceTreeItem::AddChildren(vector<ResourceNode> resources){
    for(int i=0;i<resources.size();i++){
        addChild(resources[i]);
    }
}

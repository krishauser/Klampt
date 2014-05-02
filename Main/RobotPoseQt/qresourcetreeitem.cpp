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
    setText(NAMECOL,QString::fromStdString(rt->resource->name));
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

void QResourceTreeItem::AddChildren(vector<ResourceNode> resources){
    for(int i=0;i<resources.size();i++){
        QResourceTreeItem* exp=new QResourceTreeItem(resources[i]);
        addChild(exp);
    }
}

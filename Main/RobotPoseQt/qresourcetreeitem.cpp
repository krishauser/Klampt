#include "qresourcetreeitem.h"

QResourceTreeItem::QResourceTreeItem()
{
}

//QResourceTreeItem::QResourceTreeItem(QString name,QString type){
//    setText(NAMECOL,name);
//    setText(TYPECOL,type);
//}

QResourceTreeItem::QResourceTreeItem(ResourceTracker* rt){
    resource=rt;
    setText(NAMECOL,QString::fromStdString(rt->resource->name));
    string type = rt->resource->Type();
    if(!type.empty())
        setText(TYPECOL,QString::fromStdString(rt->resource->Type()));
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

void QResourceTreeItem::AddChildren(vector<ResourceTracker*> resources){
    for(int i=0;i<resources.size();i++){
        QResourceTreeItem* exp=new QResourceTreeItem(resources[i]);
        addChild(exp);
    }
}

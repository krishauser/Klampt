#include "qresourcetreeitem.h"

QResourceTreeItem::QResourceTreeItem()
{
}

//QResourceTreeItem::QResourceTreeItem(QString name,QString type){
//    setText(NAMECOL,name);
//    setText(TYPECOL,type);
//}

QResourceTreeItem::QResourceTreeItem(ResourceTracker* tr){
    resource=tr;
    setText(NAMECOL,QString::fromStdString(tr->resource->name));
    string type = p->Type();
    if(!type.empty())
        setText(TYPECOL,QString::fromStdString(tr->resource->Type()));
    //setText(TYPECOL,"wxyz");
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

void QResourceTreeItem::AddChildren(vector<ResourceTracker*> ptrs){
    for(int i=0;i<ptrs.size();i++){
        QResourceTreeItem* exp=new QResourceTreeItem(ptrs[i]);
        addChild(exp);
    }
}

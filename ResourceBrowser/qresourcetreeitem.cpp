#include "qresourcetreeitem.h"

QResourceTreeItem::QResourceTreeItem()
{
}

//QResourceTreeItem::QResourceTreeItem(QString name,QString type){
//    setText(NAMECOL,name);
//    setText(TYPECOL,type);
//}

QResourceTreeItem::QResourceTreeItem(ResourcePtr p){
    resource=p;
    setText(NAMECOL,QString::fromStdString(resource->name));
    string type = p->Type();
    if(!type.empty())
        setText(TYPECOL,QString::fromStdString(p->Type()));
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

void QResourceTreeItem::AddChildren(vector<ResourcePtr> ptrs){
    for(int i=0;i<ptrs.size();i++){
        QResourceTreeItem* exp=new QResourceTreeItem(ptrs[i]);
        addChild(exp);
    }
}

void QResourceTreeItem::Expand()
{
    if(strcmp(resource->Type(),"Stance") == 0){
        AddChildren(ExtractResources(resource,"Hold"));
        AddChildren(ExtractResources(resource,"IKGoal"));
    }
    else if(strcmp(resource->Type(),"Hold") == 0){
        //contacts...
        AddChildren(ExtractResources(resource,"IKGoal"));
    }
    else if(strcmp(resource->Type(),"Grasp") == 0){
        AddChildren(ExtractResources(resource,"Hold"));
        AddChildren(ExtractResources(resource,"Stance"));
        AddChildren(ExtractResources(resource,"IKGoal"));
    }
    else if(strcmp(resource->Type(),"LinearPath") == 0){
        AddChildren(ExtractResources(resource,"Configs"));
    }
    else if(strcmp(resource->Type(),"MultiPath") == 0){
        AddChildren(ExtractResources(resource,"LinearPath"));
    }
    StanceResource s;
    s.stance.at(0).
}

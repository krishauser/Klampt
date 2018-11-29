#include "resourceframe.h"
#include "ui_resourceframe.h"
#include "configsframe.h"
#include "pathframe.h"
#include "floatarrayframe.h"
#include "holdframe.h"
#include "stanceframe.h"
#include "trimeshframe.h"
#include "qrobotposegui.h"

#include <QSettings>
#include <QDebug>
#include <QCoreApplication>
#include <QInputDialog>
#include <QMessageBox>


static const int numtoposertypes = 5;
static const char* toposertypes [] = {"Config","IKGoal","Stance","LinearPath","MultiPath"};
static const int numfromposertypes = 3;
static const char* fromposertypes [] = {"Config","IKGoal","Stance"};

ResourceFrame::ResourceFrame(QWidget *parent) :
  QFrame(parent), //resourceTreeModel(NULL),
    ui(new Ui::ResourceFrame)
{
    ui->setupUi(this);
}

ResourceFrame::~ResourceFrame()
{
    delete ui;
    //if(resourceTreeModel) delete resourceTreeModel;
}

void ResourceFrame::SetManager(ResourceManager* _manager)
{
  //if(resourceTreeModel) delete resourceTreeModel;
  manager = _manager;
  //resourceTreeModel = new QResourceTreeModel(_manager);
  //ui->treeWidget->setModel(resourceTreeModel);
  ui->treeWidget->setManager(manager);
  ui->treeWidget->header()->resizeSection ( 1, 80 );
  ui->treeWidget->header()->resizeSection ( 2, 20 );

  //update the Add new... box
  ui->addNewBox->clear();
  ui->addNewBox->addItem(QString("Add new..."));
  QFont font;
  font.setBold(true);
  ui->addNewBox->setItemData(0, QVariant(font), Qt::FontRole);
  //add all known types
  for(ResourceLibrary::Map::iterator i=manager->library.knownTypes.begin();i!=manager->library.knownTypes.end();i++) 
    ui->addNewBox->addItem(QString(i->first.c_str()));

  //update the stacked widget box for selected resources
  resourceToStackWidgetIndex["Configs"] = ui->selectedResourceWidget->addWidget(new ConfigsFrame(this));
  resourceToStackWidgetIndex["Stance"] = ui->selectedResourceWidget->addWidget(new StanceFrame(this));
  resourceToStackWidgetIndex["Hold"] = ui->selectedResourceWidget->addWidget(new HoldFrame(this));
  resourceToStackWidgetIndex["LinearPath"] = resourceToStackWidgetIndex["MultiPath"] = ui->selectedResourceWidget->addWidget(new PathFrame(this));
  resourceToStackWidgetIndex["TriMesh"] = ui->selectedResourceWidget->addWidget(new TriMeshFrame(this));
  resourceToStackWidgetIndex["vector<double>"] = resourceToStackWidgetIndex["vector<double>"] = ui->selectedResourceWidget->addWidget(new FloatArrayFrame(this));
  ui->selectedResourceWidget->setCurrentIndex(0);

  updateConvertBox(NULL);
  updateSelectedResourcePane(NULL);
  updateToFromPoser(NULL);
}

void ResourceFrame::OpenFile(QString filename){
    QSettings ini(QSettings::IniFormat, QSettings::UserScope,
          QCoreApplication::organizationName(),
          QCoreApplication::applicationName());  if(filename.isEmpty()){
    QFileDialog f;
    QString openDir = ini.value("last_open_resource_directory",".").toString();
    filename = f.getOpenFileName(0,"Open File",openDir,"");
  }
  if(!filename.isNull()){
    ini.setValue("last_open_resource_directory",QFileInfo(filename).absolutePath());
    //gui->SendCommand("load_resource",filename.toStdString());
    //todo send message to GUI
    ResourceNodePtr r = manager->LoadFile(filename.toStdString());
    ui->treeWidget->addNotify(r.get());
    manager->selected = r.get();
  }
}

void ResourceFrame::onResourceEdit()
{
  QTreeWidgetItem* item = ui->treeWidget->currentItem();
  if(!item) return;
  ResourceNode* n = ui->treeWidget->itemToNode(item);
  assert(n != NULL);
  n->SetChanged();
  if(n->IsExpanded()) {
    //need to re-expand
    if(!n->IsValid()) {
      QMessageBox::warning(this,"Edit warning","Resource's children have not been backed up, edit will delete changes");
    }
    while(item->childCount() > 0) 
      ui->treeWidget->onDelete(item->child(0));
    ui->treeWidget->onExpand(item);
  }
  ui->treeWidget->updateProperties(item);
  ui->treeWidget->updateDecorator(item);
  gui->backend->SendRefresh();
}

bool ResourceFrame::doBackup(QTreeWidgetItem* it)
{
  ResourceNode* n = ui->treeWidget->itemToNode(it);
  bool res = true;
  string errormessage;
  ResourceNode* where;
  if(!n->Backup(&errormessage,&where)) {
    string title = "Couldn't back up changes to "+where->Identifier();
    string msg = errormessage;
    QMessageBox::warning(this,title.c_str(),QString(msg.c_str()));
    res = false;
  }
  else {
    //printf("Backup successful\n");
    res = true;
  }
  ui->treeWidget->updateAllDecorators();
  return res;
}

void ResourceFrame::updateConvertBox(ResourcePtr resource)
{
  ui->convertToBox->clear();
  ui->convertToBox->addItem(QString("Convert to..."));
  QFont font;
  font.setBold(true);
  ui->convertToBox->setItemData(0, QVariant(font), Qt::FontRole);
  if(!resource) {
    ui->convertToBox->setEnabled(false);
    return;
  }
  //loop through all possible conversions
  vector<string> types = CastResourceTypes(resource);
  for(size_t i=0;i<types.size();i++) {
    ui->convertToBox->addItem(types[i].c_str());
  }
  if(types.empty()) 
    ui->convertToBox->setEnabled(false);
  else
    ui->convertToBox->setEnabled(true);
}

void ResourceFrame::updateToFromPoser(ResourcePtr resource)
{
  if(!resource) {
    ui->btn_togui->setEnabled(false);
    ui->btn_fromgui->setEnabled(false);
    return;
  }
  bool enablefrom=false,enableto=false;
  const char* type = resource->Type();
  for(int i=0;i<numfromposertypes;i++)
    if(0==strcmp(fromposertypes[i],type)) {
      enablefrom=true;
      break;
    }
  for(int i=0;i<numtoposertypes;i++)
    if(0==strcmp(toposertypes[i],type)) {
      enableto=true;
      break;
    }
  ui->btn_togui->setEnabled(enableto);
  ui->btn_fromgui->setEnabled(enablefrom);
}

void ResourceFrame::updateSelectedResourcePane(ResourcePtr resource)
{
  if(!resource) return;
  if(resourceToStackWidgetIndex.count(resource->Type()) == 0) {
    //printf("Setting selection index 0 for type %s\n",resource->Type());
    ui->selectedResourceWidget->setCurrentIndex(0);
  }
  else {
    int index=resourceToStackWidgetIndex[resource->Type()];
    //printf("Setting selection index %d for type %s\n",index,resource->Type());
    QWidget* w = ui->selectedResourceWidget->widget(index);
    if(0==strcmp(resource->Type(),"Configs")) 
      dynamic_cast<ConfigsFrame*>(w)->set(resource.get());
    else if(0==strcmp(resource->Type(),"Hold"))
      dynamic_cast<HoldFrame*>(w)->set(resource.get());
    else if(0==strcmp(resource->Type(),"Stance"))
      dynamic_cast<StanceFrame*>(w)->set(resource.get());
    else if(0==strcmp(resource->Type(),"vector<double>"))
      dynamic_cast<FloatArrayFrame*>(w)->set(resource.get());
    else if(0==strcmp(resource->Type(),"LinearPath"))
      dynamic_cast<PathFrame*>(w)->set(resource.get());
    else if(0==strcmp(resource->Type(),"MultiPath"))
      dynamic_cast<PathFrame*>(w)->set(resource.get());
    else if(0==strcmp(resource->Type(),"TriMesh"))
      dynamic_cast<TriMeshFrame*>(w)->set(resource.get());
    else
      fprintf(stderr,"Uh... not handling edit widget for resource of type %s\n",resource->Type());
    ui->selectedResourceWidget->setCurrentIndex(index);
  }
}

void ResourceFrame::ChangeSelectedItem(QTreeWidgetItem* it){
  if(!it) {
    //disable all editors
    updateConvertBox(NULL);
    return;
  }
  manager->selected = ui->treeWidget->itemToNode(it);

  if(manager->selected) {
    //attempt backup if selected
    if(manager->selected->IsDirty()) {
      doBackup(it);
    }

    updateConvertBox(manager->selected->resource);
    updateSelectedResourcePane(manager->selected->resource);
    updateToFromPoser(manager->selected->resource);
  }
  gui->backend->SendRefresh();
}

void ResourceFrame::PressedDelete(){
  ui->treeWidget->onDeletePressed();

  /*
  //delete from model
  ResourceNode* n = resourceTreeModel->getItem(index);
  const QModelIndex index = treeView->selectionModel()->currentIndex();
  int row = manager->ChildIndex(n);
  resourceTreeModel->removeRows(index.parent(),row,1);
  //delete from manager
  if(n->parent) {
    n->SetChanged();
    n->parent->children.erase(n->parent->children.begin()+row);
  }
  else {
    library.Erase(n->resource);
    manager->topLevel.erase(manager->topLevel.begin()+row);
  }
  */
}

void ResourceFrame::ItemChanged(QTreeWidgetItem* sel,int col)
{
  //qDebug()<<"ItemChanged: "<<(int)sel;
  if(col != NAMECOL) {
    qDebug()<<"Can only edit name column";
    //restore
    return;
  }
  //qDebug()<<"Changing name to: "<<sel->text(NAMECOL);
  ui->treeWidget->itemToNode(sel)->resource->name = sel->text(NAMECOL).toStdString();
}

void ResourceFrame::ItemExpanded(QTreeWidgetItem* sel)
{
  ui->treeWidget->onExpand(sel);  
  /*
  const QModelIndex index = treeView->selectionModel()->currentIndex();
  ResourceNode* node = resourceTreeModel.getItem(index);
  if(node && !node->IsExpanded()) {
    node->Expand();
    for(size_t i=0;i<node->children.size();i++)
      resourceTreeModel->addNotify(node->children[i]);
  }
  */
}

/*
void ResourceFrame::DiscretizePath()
{
    int value = QInputDialog::getInteger(this,"Discrete Segments","Enter the number of configurations to generate",10,1);
    if(value > 0){
        gui->SendCommand("discretize_path",value);
    }
}
*/


void ResourceFrame::SaveResource()
{
  QSettings ini(QSettings::IniFormat, QSettings::UserScope,
		QCoreApplication::organizationName(),
		QCoreApplication::applicationName());

  QTreeWidgetItem* item = ui->treeWidget->currentItem();
  if(!item) return;
  doBackup(item);
  ResourceNode* n = ui->treeWidget->itemToNode(item);

  QFileDialog f;
  //do we want to go from registry ?
  QString openDir;
  bool changeSettings = true;
  if(!n->resource->fileName.empty()) {
    changeSettings = false;
    openDir = QString(n->resource->fileName.c_str());
  }
  else {
    QString defaultfn = QString::fromStdString(manager->library.DefaultFileName(n->resource));
    openDir = QDir(ini.value("save_resource_dir",".").toString()).filePath(defaultfn);
    changeSettings = true;
  }
  //setup extension filter by resource type
  QString filter;
  for(ResourceLibrary::Map::const_iterator i=manager->library.loaders.begin();i!=manager->library.loaders.end();i++) {
    for(size_t k=0;k<i->second.size();k++) {
      if(0==strcmp(n->resource->Type(),i->second[k]->Type())) {
	if(!filter.isEmpty())
	  filter += QString(" ");
	filter += QString("*.")+QString::fromStdString(i->first);
      }
    }
  }
  if(!filter.isEmpty()) {
    filter = QString(n->resource->Type())+QString(" files (")+filter+QString(")");
    filter += QString(";;All files (*.*)");
  }
  //Do the file name
  QString filename = f.getSaveFileName(0,"Save To...",openDir,filter);
  if(!filename.isEmpty()){
    //save to registry
    if(changeSettings) 
      ini.setValue("save_resource_dir",f.directory().absolutePath());

    manager->Save(n,filename.toStdString());
    ui->treeWidget->updateDecorator(item);
  }
}

void ResourceFrame::SaveAllResources()
{
  QSettings ini(QSettings::IniFormat, QSettings::UserScope,
		QCoreApplication::organizationName(),
		QCoreApplication::applicationName());
  QFileDialog f;
  QString openDir = ini.value("save_resource_folder",".").toString();
  QString filename = f.getExistingDirectory(this,"Save Library To...",openDir,QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  if(!filename.isEmpty()){
    //save to registry
    ini.setValue("save_resource_folder",QFileInfo(filename).absolutePath());

    manager->SaveFolder(filename.toStdString());
    ui->treeWidget->updateAllDecorators();
  }
}

void ResourceFrame::ToGUI(){
    gui->SendCommand("resource_to_poser");
}

void ResourceFrame::AddNew(QString type){
    if(type == "Add new..." || type == "") return;
    gui->SendCommand("add_resource",type.toStdString(),"\"\"");
    ui->addNewBox->setCurrentIndex(0);
}

void ResourceFrame::ConvertTo(QString type)
{
  if(type == "Convert to..." || type == "") return;
    gui->SendCommand("convert",type.toStdString());
  ui->addNewBox->setCurrentIndex(0);
}

void ResourceFrame::FromGUI(){
  if(manager->selected) {
    gui->SendCommand("poser_to_resource_overwrite");
    QTreeWidgetItem* item = ui->treeWidget->currentItem();
    if(item) {
      if(item->parent()!=NULL) {
	ui->treeWidget->updateDecorator(item->parent());
      }
    }
  }
}


void ResourceFrame::updateNewResource(string id)
{
  ResourceNodePtr node = manager->Get(id);
  if(node == NULL) {
    cout<<"updateNewResource: "<<id<<" doesn't exist"<<endl;
    manager->Print();
    return;
  }
  assert(node != NULL);
  ui->treeWidget->addNotify(node.get());
}

void ResourceFrame::refreshResources()
{
  ui->treeWidget->refresh();
}

void ResourceFrame::SendPathTime(double t){
    gui->SendCommand("set_path_time",t);
}


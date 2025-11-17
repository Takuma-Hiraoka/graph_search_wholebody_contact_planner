#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <choreonoid_viewer/choreonoid_viewer.h>

namespace graph_search_wholebody_contact_planner_sample{
  void sample0_locomotion();
  class sample0_locomotionItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample0_locomotionItem>("sample0_locomotionItem"); }
  protected:
    virtual void main() override{ sample0_locomotion(); return;}
  };
  typedef cnoid::ref_ptr<sample0_locomotionItem> sample0_locomotionItemPtr;

  void sample1_manipulation();
  class sample1_manipulationItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample1_manipulationItem>("sample1_manipulationItem"); }
  protected:
    virtual void main() override{ sample1_manipulation(); return;}
  };
  typedef cnoid::ref_ptr<sample1_manipulationItem> sample1_manipulationItemPtr;

  void sample2_locomanipulation();
  class sample2_locomanipulationItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample2_locomanipulationItem>("sample2_locomanipulationItem"); }
  protected:
    virtual void main() override{ sample2_locomanipulation(); return;}
  };
  typedef cnoid::ref_ptr<sample2_locomanipulationItem> sample2_locomanipulationItemPtr;

  class GraphSearchWholeBodyContactPlannerSamplePlugin : public cnoid::Plugin
  {
  public:
    GraphSearchWholeBodyContactPlannerSamplePlugin() : Plugin("GraphSearchWholeBodyContactPlannerSample")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      sample0_locomotionItem::initializeClass(this);
      sample1_manipulationItem::initializeClass(this);
      sample2_locomanipulationItem::initializeClass(this);
      return true;
    }
  };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(graph_search_wholebody_contact_planner_sample::GraphSearchWholeBodyContactPlannerSamplePlugin)

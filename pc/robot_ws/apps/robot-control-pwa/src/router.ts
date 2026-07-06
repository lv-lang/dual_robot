import { createRouter, createWebHistory, type RouteRecordRaw } from "vue-router";
import OverviewView from "./views/OverviewView.vue";
import TasksView from "./views/TasksView.vue";
import ConfirmationView from "./views/ConfirmationView.vue";
import LogsView from "./views/LogsView.vue";
import SystemView from "./views/SystemView.vue";

export const routes: RouteRecordRaw[] = [
  { path: "/", name: "overview", component: OverviewView },
  { path: "/tasks", name: "tasks", component: TasksView },
  { path: "/confirmations", name: "confirmations", component: ConfirmationView },
  { path: "/logs", name: "logs", component: LogsView },
  { path: "/system", name: "system", component: SystemView }
];

export const router = createRouter({
  history: createWebHistory(),
  routes
});

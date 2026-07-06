<script setup lang="ts">
import { computed, reactive, ref, watch } from "vue";
import type { BusinessPoint, TaskTemplate, TaskType, TemplatePayload } from "../types";

const props = defineProps<{
  template?: TaskTemplate;
  businessPoints: BusinessPoint[];
  saving?: boolean;
}>();

const emit = defineEmits<{
  save: [payload: TemplatePayload];
  cancel: [];
}>();

const form = reactive({
  name: "",
  task_type: "DELIVERY" as TaskType,
  robot_preference: "auto" as "auto" | "mecanum" | "ackermann",
  pickup: "",
  delivery: "",
  inspections: [] as string[],
  sort_order: 100
});

const pickupPoints = computed(() =>
  props.businessPoints.filter((point) => point.point_type === "pickup")
);
const deliveryPoints = computed(() =>
  props.businessPoints.filter((point) => point.point_type === "delivery")
);
const inspectionPoints = computed(() =>
  props.businessPoints.filter((point) => point.point_type === "inspection")
);

function hasPoint(pointId: string, pointType: BusinessPoint["point_type"]): boolean {
  return props.businessPoints.some((point) => point.point_id === pointId && point.point_type === pointType);
}

function normalizeDeliveryFields(): void {
  form.inspections = [];
  if (!hasPoint(form.pickup, "pickup")) {
    form.pickup = pickupPoints.value[0]?.point_id ?? "";
  }
  if (!hasPoint(form.delivery, "delivery")) {
    form.delivery = deliveryPoints.value[0]?.point_id ?? "";
  }
}

function normalizeInspectionFields(): void {
  form.pickup = "";
  form.delivery = "";
  form.inspections = form.inspections.filter((pointId) => hasPoint(pointId, "inspection"));
}

function pointOptionLabel(point: BusinessPoint): string {
  const label = point.label.trim();
  if (!label || label === point.point_id) {
    return point.point_id;
  }
  return `${point.point_id} · ${label}`;
}

const validationError = computed(() => {
  if (!form.name.trim()) {
    return "请输入模板名称";
  }
  if (
    form.task_type === "DELIVERY" &&
    (!hasPoint(form.pickup, "pickup") || !hasPoint(form.delivery, "delivery"))
  ) {
    return "配送任务需要一个取货点和一个配送点";
  }
  if (
    form.task_type === "INSPECTION" &&
    (form.inspections.length === 0 || form.inspections.some((pointId) => !hasPoint(pointId, "inspection")))
  ) {
    return "巡检任务至少需要一个巡检点";
  }
  return undefined;
});

const dirtyKey = ref(0);

watch(
  () => props.template,
  (template) => {
    form.name = template?.name ?? "";
    form.task_type = template?.task_type ?? "DELIVERY";
    form.robot_preference = template?.robot_preference ?? "auto";
    form.pickup = template?.task_type === "DELIVERY" ? template.target_points[0] ?? "" : "";
    form.delivery = template?.task_type === "DELIVERY" ? template.target_points[1] ?? "" : "";
    form.inspections = template?.task_type === "INSPECTION" ? [...template.target_points] : [];
    form.sort_order = template?.sort_order ?? 100;
    if (form.task_type === "DELIVERY") {
      normalizeDeliveryFields();
    } else {
      normalizeInspectionFields();
    }
    dirtyKey.value += 1;
  },
  { immediate: true }
);

watch(
  () => form.task_type,
  (taskType) => {
    if (taskType === "DELIVERY") {
      normalizeDeliveryFields();
    } else {
      normalizeInspectionFields();
    }
  }
);

watch(
  () => props.businessPoints,
  () => {
    if (form.task_type === "DELIVERY") {
      normalizeDeliveryFields();
    } else {
      normalizeInspectionFields();
    }
  },
  { deep: true }
);

function submit(): void {
  if (validationError.value) {
    return;
  }
  emit("save", {
    name: form.name.trim(),
    task_type: form.task_type,
    robot_preference: form.robot_preference,
    target_points:
      form.task_type === "DELIVERY" ? [form.pickup, form.delivery] : [...form.inspections],
    sort_order: Number(form.sort_order)
  });
}
</script>

<template>
  <form class="panel template-form" @submit.prevent="submit">
    <div class="page-heading">
      <h2 class="section-title">{{ template ? "编辑模板" : "新建自定义模板" }}</h2>
      <button class="button-secondary" type="button" @click="emit('cancel')">关闭</button>
    </div>

    <div class="form-grid">
      <div class="field">
        <label for="template-name">名称</label>
        <input id="template-name" v-model="form.name" autocomplete="off" />
      </div>

      <div class="field">
        <label for="template-type">任务类型</label>
        <select id="template-type" v-model="form.task_type" :key="dirtyKey">
          <option value="DELIVERY">配送</option>
          <option value="INSPECTION">巡检</option>
        </select>
      </div>

      <div class="field">
        <label for="template-robot">机器人偏好</label>
        <select id="template-robot" v-model="form.robot_preference">
          <option value="auto">自动</option>
          <option value="mecanum">mecanum</option>
          <option value="ackermann">ackermann</option>
        </select>
      </div>

      <div class="field">
        <label for="template-order">排序</label>
        <input id="template-order" v-model.number="form.sort_order" type="number" min="0" step="10" />
      </div>

      <template v-if="form.task_type === 'DELIVERY'">
        <div class="field">
          <label for="pickup-point">取货点</label>
          <select id="pickup-point" v-model="form.pickup">
            <option v-for="point in pickupPoints" :key="point.point_id" :value="point.point_id">
              {{ pointOptionLabel(point) }}
            </option>
          </select>
        </div>

        <div class="field">
          <label for="delivery-point">配送点</label>
          <select id="delivery-point" v-model="form.delivery">
            <option v-for="point in deliveryPoints" :key="point.point_id" :value="point.point_id">
              {{ pointOptionLabel(point) }}
            </option>
          </select>
        </div>
      </template>

      <div v-else class="field full">
        <label>巡检点</label>
        <div class="checkbox-grid">
          <label v-for="point in inspectionPoints" :key="point.point_id" class="checkbox-tile">
            <input v-model="form.inspections" type="checkbox" :value="point.point_id" />
            <span>{{ pointOptionLabel(point) }}</span>
          </label>
        </div>
      </div>
    </div>

    <p v-if="validationError" class="notice error">{{ validationError }}</p>

    <div class="button-row">
      <button type="submit" :disabled="Boolean(validationError) || saving">
        {{ template ? "保存模板" : "创建模板" }}
      </button>
      <button class="button-secondary" type="button" :disabled="saving" @click="emit('cancel')">
        取消
      </button>
    </div>
  </form>
</template>

<style scoped>
.template-form {
  display: grid;
  gap: 14px;
}
</style>

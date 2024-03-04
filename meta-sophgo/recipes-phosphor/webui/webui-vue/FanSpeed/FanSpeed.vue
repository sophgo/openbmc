<template>
  <b-container fluid="xl">
    <page-title :description="$t('pageFanSpeed.description')" />

    <b-row>
      <b-col sm="8" md="6" xl="12">
        <b-form-group :label="$t('pageFanSpeed.policiesLabel.Manul')">
          <b-form-radio-group
            v-model="currentFanSpeedControlPolicy"
            name="fanspeed-control-policy"
          >
          <b-form-radio value="low">{{ $t('pageFanSpeed.gradeDesc.Low') }}</b-form-radio>
          <b-form-radio value="medium">{{ $t('pageFanSpeed.gradeDesc.Medium') }}</b-form-radio>
          <b-form-radio value="high">{{ $t('pageFanSpeed.gradeDesc.High') }}</b-form-radio>
          </b-form-radio-group>
        </b-form-group>
      </b-col>
    </b-row>

    <b-row>
      <b-col sm="8" md="6" xl="12">
        <b-form-group :label="$t('pageFanSpeed.policiesLabel.Auto')">
          <b-form-radio-group
            v-model="currentFanSpeedControlPolicy"
            name="fanspeed-control-policy"
          >
          <b-form-radio value="auto">{{ $t('pageFanSpeed.gradeDesc.Auto') }}</b-form-radio>
          </b-form-radio-group>
        </b-form-group>
      </b-col>
    </b-row>

    <b-button variant="primary" type="submit" @click="submitForm">
      {{ $t('global.action.saveSettings') }}
    </b-button>
  </b-container>
</template>

<script>
import PageTitle from '@/components/Global/PageTitle';
import BVToastMixin from '@/components/Mixins/BVToastMixin';
import LocalTimezoneLabelMixin from '@/components/Mixins/LocalTimezoneLabelMixin';
import VuelidateMixin from '@/components/Mixins/VuelidateMixin.js';
import LoadingBarMixin from '@/components/Mixins/LoadingBarMixin';

export default {
  name: 'FanSpeed',
  components: { PageTitle },
  mixins: [
    BVToastMixin,
    LoadingBarMixin,
    LocalTimezoneLabelMixin,
    VuelidateMixin,
  ],
  beforeRouteLeave(to, from, next) {
    this.hideLoader();
    next();
  },
  data() {
    return {
      policyValue: null,
    };
  },

  computed: {
    currentFanSpeedControlPolicy: {
      get() {
        return this.$store.getters['fanSpeed/fanSpeedPolicy'];
      },
      set(policy) {
        this.policyValue = policy;
      },
    },
  },
  created() {
    this.startLoader();
    this.renderFanSpeedSettings();
  },
  methods: {
    renderFanSpeedSettings() {
      Promise.all([
        this.$store.dispatch('fanSpeed/getFanSpeed'),
      ]).finally(() => {
        this.endLoader();
      });
    },
    submitForm() {
      this.startLoader();
      this.$store
      .dispatch(
        'fanSpeed/setFanSpeed',
        this.policyValue
      )
      .finally(() => {
          this.renderFanSpeedSettings();
      });
    },
  },
};
</script>

<template>
  <b-container fluid="xl">
    <page-title />

    <page-section :section-title="$t('pageFanSpeed.configureSettings')">
      <b-form novalidate @submit.prevent="submitForm">
        <b-form-group
          label="Configure fan speed"
          label-sr-only
        >
          <b-form-radio
            v-model="form.configurationSelected"
            value="manual"
            data-test-id="fanSpeed-radio-configureManual"
          >
            {{ $t('pageFanSpeed.form.manual') }}
          </b-form-radio>
          <b-row class="mt-3 ml-3">
            <b-col sm="6" lg="4" xl="3">
              <b-form-group
                :label="$t('pageFanSpeed.form.pwm')"
                label-for="input-manual-pwm"
              >
                <b-form-text id="pwm-format-help">0-255</b-form-text>
                <b-input-group>
                  <b-form-input
                    id="input-manual-pwm"
                    type="number"
                    v-model.number="form.manual.pwm"
                    min="0"
                    max="255"
                    :disabled="autoOptionSelected"
                    @input="validateInput"
                  />
                </b-input-group>
              </b-form-group>
            </b-col>
          </b-row>
          <b-form-radio
            v-model="form.configurationSelected"
            value="auto"
            data-test-id="fanSpeed-radio-configureAuto"
          >
            Auto
          </b-form-radio>
          <b-button
            variant="primary"
            type="submit"
            data-test-id="fanSpeed-button-saveSettings"
          >
            {{ $t('global.action.saveSettings') }}
          </b-button>
        </b-form-group>
      </b-form>
    </page-section>
  </b-container>
</template>

<script>
import PageTitle from '@/components/Global/PageTitle';
import PageSection from '@/components/Global/PageSection';

import BVToastMixin from '@/components/Mixins/BVToastMixin';
import LoadingBarMixin, { loading } from '@/components/Mixins/LoadingBarMixin';
import LocalTimezoneLabelMixin from '@/components/Mixins/LocalTimezoneLabelMixin';
import VuelidateMixin from '@/components/Mixins/VuelidateMixin.js';


export default {
  name: 'FanSpeed',
  components: { PageTitle, PageSection },
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
      locale: this.$store.getters['global/languagePreference'],
      form: {
        configurationSelected: 'manual',
        manual: {
          pwm: 255,
        },
        auto: '',
      },
      loading,
    };
  },
  computed: {
    autoOptionSelected() {
      return this.form.configurationSelected === 'auto';
    },
  },
  // created() {
  //   this.startLoader();
  // },
  methods: {
    validateInput() {
      if (this.form.manual.pwm < 0) {
        this.form.manual.pwm = 0;
      } else if (this.form.manual.pwm > 255) {
        this.form.manual.pwm = 255;
      }
    },
    submitForm() {
      this.startLoader();
      let pwmString;
      let isAUTOEnabled = this.form.configurationSelected === 'auto';
      //manual
      if (!isAUTOEnabled) {
        // console.log('111');
        pwmString = this.form.manual.pwm.toString();
      } else { //auto
        // console.log('222');
        pwmString = "auto";
      }
      console.log('333:',pwmString);
      this.$store
      .dispatch('fanSpeed/setFanSpeed', pwmString);
      this.endLoader();
    },
  },
};
</script>

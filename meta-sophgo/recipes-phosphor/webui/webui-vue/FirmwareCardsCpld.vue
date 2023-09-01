<template>
  <page-section :section-title="$t('pageFirmware.sectionTitleCpldCards')">
    <b-card-group deck>
      <!-- Running image -->
      <b-card>
        <template #header>
          <p class="font-weight-bold m-0">
            {{ $t('pageFirmware.cardTitleRunning') }}
          </p>
        </template>
        <dl class="mb-0">
          <dt>{{ $t('pageFirmware.cardBodyVersion') }}</dt>
          <dd class="mb-0">{{ runningVersion }}</dd>
        </dl>
      </b-card>

      <!-- Backup image -->
      <b-card>
        <template #header>
          <p class="font-weight-bold m-0">
            {{ $t('pageFirmware.cardTitleBackup') }}
          </p>
        </template>
        <dl class="mb-0">
          <dt>{{ $t('pageFirmware.cardBodyVersion') }}</dt>
          <dd class="mb-0">
            {{ backupVersion }}
          </dd>
        </dl>
      </b-card>
    </b-card-group>
  </page-section>
</template>

<script>
import PageSection from '@/components/Global/PageSection';

export default {
  components: { PageSection },
  computed: {
    // running() {
    //   return this.$store.getters['firmware/activeCpldFirmware'];
    // },
    // runningVersion() {
    //   return this.running?.version || '--';
    // },
    backup() {
      return this.$store.getters['firmware/backupHostFirmware'];
    },
    runningVersion() {
      return this.$store.getters['global/cpldVersion'];
    },
    backupVersion() {
      return this.backup?.version || '--';
    },
  },
};
</script>

<style lang="scss" scoped>
.page-section {
  margin-top: -$spacer * 1.5;
}
</style>

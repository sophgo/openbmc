import api from '@/store/api';
import i18n from '@/i18n';

const FanSpeedStore = {
  namespaced: true,
  actions: {
    async setFanSpeed(_, pwm) {
      const data = { FanSpeed: pwm };

      return await api
        .patch('/redfish/v1/Systems/system', data)
        .then(() => {
          return i18n.t('pageFanSpeed.toast.successSaveSettings');
        })
        .catch((error) => {
          console.log(error);
          throw new Error(
            i18n.t('pageFanSpeed.toast.errorSaveSettings')
          );
        });
    },
  },
};

export default FanSpeedStore;

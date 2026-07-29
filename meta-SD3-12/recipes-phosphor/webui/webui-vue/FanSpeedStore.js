import api from '@/store/api';
import i18n from '@/i18n';

const FanSpeedStore = {
  namespaced: true,
  state: {
    fanSpeedPolicy: null,
  },
  getters: {
    fanSpeedPolicy: (state) => state.fanSpeedPolicy,
  },
  mutations: {
    setFanSpeedCurrentPolicy: (state, fanSpeedPolicy) =>
      (state.fanSpeedPolicy = fanSpeedPolicy),
  },
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

    async getFanSpeed({ commit }) {
      return await api
        .get('/redfish/v1/Systems/system')
        .then(({ data: { FanSpeed } }) => {
          commit('setFanSpeedCurrentPolicy', FanSpeed);
        })
        .catch((error) => console.log(error));
    },
  },
};

export default FanSpeedStore;

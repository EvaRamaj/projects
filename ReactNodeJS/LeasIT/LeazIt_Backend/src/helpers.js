const Role = require('./role_constants');

const getRole = (checkRole) => {
  let role;

  switch (checkRole) {
    case Role.ROLE_ADMIN: role = 3; break;
    case Role.ROLE_LESSOR: role = 2; break;
    case Role.ROLE_LESSEE: role = 1; break;
    default: role = 1;
  }

  return role;
};

module.exports = {
    getRole
};

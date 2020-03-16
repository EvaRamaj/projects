"use strict";

import React from 'react';
import PropTypes from 'prop-types';
import { MenuButton, ListItem, Avatar, FontIcon } from 'react-md';
import { withRouter } from 'react-router-dom'

import AuthService from '../services/AuthService';


class KebabMenu extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            user: AuthService.isAuthenticated() ? AuthService.getCurrentUser() : undefined
        }
    }

    logout() {
        AuthService.logout();
        this.state = {
            user: AuthService.isAuthenticated() ? AuthService.getCurrentUser() : undefined
        };
        if(this.props.location.pathname != '/') {
            this.props.history.push('/');
        }
        else {
            window.location.reload();
        }
    }

    render() {
        return (
            <MenuButton
                id={this.props.id}
                icon
                className={this.props.className}
                menuItems={this.state.user ? [
                    <ListItem key={1} leftAvatar={<Avatar icon={<FontIcon>account_circle</FontIcon>}/>} primaryText={this.state.user.username} onClick={() => this.props.history.push('/my_profile')}/>,
                    <ListItem key={3} primaryText="Logout" onClick={() => this.logout()}/>
                ]: [<ListItem key={1} primaryText="Login" onClick={() => this.props.history.push('/login')}/>]}
            >
                more_vert
            </MenuButton>
        );
    }
}

KebabMenu.propTypes = {
    id: PropTypes.string.isRequired,
    className: PropTypes.string,
    menuItems: PropTypes.array
};

export default withRouter(KebabMenu);
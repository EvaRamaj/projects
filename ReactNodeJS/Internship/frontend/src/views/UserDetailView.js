"use strict";

import React from 'react';

import UserDetails from '../components/UserDetails';
import UserService from '../services/UserService';


export class UserDetailView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            loading: true,
            user: []
        }
    }

    componentWillMount(props){
        this.setState({
            loading: true
        });
        let id = this.props.match.params.id;
        console.log(id);
        UserService.getUser(id).then((data) => {
            this.setState({
                user: data,
                loading: false
            });
        }).catch((e) => {
            console.error(e);
        });

    }

    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }

        return (
            <UserDetails user={this.state.user} />
        );
    }
}

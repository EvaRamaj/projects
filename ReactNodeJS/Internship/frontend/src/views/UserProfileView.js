"use strict";

import React from 'react';
import ReactLoading from 'react-loading';

import UserProfile from '../components/UserProfile';

import UserService from '../services/UserService';


export class UserProfileView extends React.Component {

    constructor(props) {
        super(props);
    }

    componentWillMount(props)
    {
        this.setState({
            loading: true,
            user: undefined,
            didSubmit: false
        });
    }

    componentDidMount()
    {
        this.sleep(1000).then(() =>
            UserService.getCurrentUserDetails().then((data) => {
            this.setState({
                loading: false,
                user: data
            })
        }).catch((e) => {
            console.error(e);
        }))
    }

    sleep(milliseconds)
    {
        return new Promise(resolve => setTimeout(resolve, milliseconds))
    }

    requestUpgrade(user) {
        UserService.requestUpgrade(user).then((data) => {
            this.setState({
                didSubmit: true
            })
        }).catch((e) => {
            console.error(e);
            this.setState({
                error: e
            });
        });
    }


    render() {
        if (this.state.loading) {
            return (
                <div style={{marginTop:200+'px'}}>
                    <div className="col-md-5"/>
                    <div className="col-md-2">
                        <ReactLoading type="spinningBubbles" color="#000000" height={500} width={200}/>
                    </div>
                    <div className="col-md-5"/>
                </div>
            );
        }

        return (
            <UserProfile user={this.state.user} didSubmit={this.state.didSubmit} onSubmit={(user) => this.requestUpgrade(user)} error={this.state.error}/>
        );
    }
}
